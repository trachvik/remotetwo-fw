/*
 * anticogging.c
 * Final Consistent Calibration (Lerp + Single-Pass Filter)
 */

/* TO DO: alghoritm still isn't 100% reliable - occasional oscillation resulting in the wrong LUT values.
   hardcoded LUT -> directly write LUT to the flash memory and read it from there.
 */

#include "anticogging.h"
#include "drivers/bldc_motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <pm_config.h>
#include <errno.h>
#include <math.h>

extern float sensor_get_angle(void *sensor);
extern float sensor_get_velocity(void *sensor);
extern void sensor_update(void *sensor);

LOG_MODULE_REGISTER(anticogging, LOG_LEVEL_INF);

#define SAMPLE_COUNT 500
#define TWO_PI 6.28318530718f
#define CALIB_SPEED_RAD_S 3.14159f
#define CALIB_REVOLUTIONS 3
#define MAX_ITERATIONS 5
/* Learning gain per iteration. The residual measured in iteration 1 (with ff=0)
 * IS the full current the velocity loop needs to overcome cogging, so we apply
 * it fully (1.0). A value < 1.0 only makes sense if the ILC truly converges over
 * many iterations - here it doesn't (measurement lag), we keep the best single
 * pass, so 0.5 would just ship half the real cogging (under-compensation). */
#define RELAXATION_GAIN 1.0f
#define ERROR_THRESHOLD 0.005f 
/* Number of independent, NON-compensating calibration passes to average. Each
 * pass measures the raw (ff=0) cogging, so all passes estimate the SAME fixed
 * quantity. Averaging them cuts random stick-slip / sensor noise by ~sqrt(N)
 * without the systematic bias that iterative feedback added at sharp features. */
#define CALIB_AVERAGES 3
/* Break-away current pre-loaded into the velocity-PID integrator at the start of
 * each pass. Gives the rotor immediate torque to overcome static friction/cogging
 * so the velocity set-point can ramp smoothly from 0 (no start step = no lurch). */
#define CALIB_BREAKAWAY_A 0.12f

/* ---- Option B: standstill DETENT calibration ----------------------------------
 * Set to 1 to run a one-time standstill calibration that measures the STATIC
 * detent you feel by hand (correct amplitude AND phase, unlike the spinning map)
 * and DUMPS the LUT to serial. Paste that dump into anticogging_data.c, then set
 * this back to 0. */
#define ANTICOGGING_STATIC_CALIB 1
#define STATIC_POINTS      500     /* one hold target per LUT bin (full coverage) */
#define STATIC_SETTLE_MS   250     /* let the rotor settle before measuring */
#define STATIC_MEASURE_MS  60      /* window whose mean holding current = detent */
#define STATIC_FOC_HZ      5000.0f
#define STATIC_POS_HZ      500.0f  /* must match TMAG/PLL update, not the 5 kHz FOC rate */

/* OPRAVENÉ ZISKY PRO FOC_CURRENT MÓD BEZ VIBRACÍ: */
#define STATIC_POS_KP      0.50f   /* Zlatý střed - dost silné, ale neagresivní */
#define STATIC_POS_KI      2.5f    /* Zajišťuje utržení ze zubů */
#define STATIC_POS_KD      0.0f    /* MUSÍ BÝT STRIKTNĚ NULA - odstraní vysokofrekvenční vibrace! */
#define STATIC_VEL_ALPHA   0.08f
#define STATIC_HOLD_CURRENT_LIMIT 0.50f

/* ---- Calibration algorithm selector -------------------------------------------
 * 1 = iterative position-hold (Piccoli & Yim Position-Based method + Qian ILC
 *     relaxation): hold each angle at standstill, inject the current LUT as FF,
 *     measure the filtered holding current, relax the LUT toward it each pass.
 *     Standstill -> no current-loop phase lag -> best for haptics on this HW.
 * 0 = single-pass static detent hold (same physics, one forward+backward sweep). */
#define ANTICOGGING_USE_ILC 1
#define ILC_ITERATIONS     4       /* refinement passes; PI residual -> 0 as LUT converges */
#define ILC_RELAX_GAIN     0.6f    /* relaxation: lut += gain*(measured - lut) */
#define STATIC_DT          (1.0f / STATIC_FOC_HZ)

static float final_lut[SAMPLE_COUNT] = {0};

float anticogging_get_feedforward(float mechanical_angle) 
{
    if (isnan(mechanical_angle) || isinf(mechanical_angle)) return 0.0f;
    
    float norm_angle = fmodf(mechanical_angle, TWO_PI);
    if (norm_angle < 0.0f) norm_angle += TWO_PI;
    
    float float_index = (norm_angle / TWO_PI) * (float)SAMPLE_COUNT;
    int idx1 = (int)float_index;
    
    /* BEZPEČNOST: Ochrana proti přetečení indexu na hranici otáčky */
    if (idx1 >= SAMPLE_COUNT) idx1 = SAMPLE_COUNT - 1;
    if (idx1 < 0) idx1 = 0;
    
    int idx2 = (idx1 + 1) % SAMPLE_COUNT; 
    float fraction = float_index - (float)idx1;
    
    return final_lut[idx1] * (1.0f - fraction) + final_lut[idx2] * fraction;
}

/* Circular (wrap-around) moving-average smoothing of a LUT in place. `window`
 * should be odd. Used to reject random per-bin spikes so the feedforward - and
 * therefore the motor motion - stays smooth. */
static void __attribute__((unused))
smooth_lut_circular(float *arr, int window)
{
    static float tmp[SAMPLE_COUNT];
    int half = window / 2;
    int n = 2 * half + 1;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float sum = 0.0f;
        for (int j = -half; j <= half; j++) {
            int idx = (i + j + SAMPLE_COUNT) % SAMPLE_COUNT;
            sum += arr[idx];
        }
        tmp[i] = sum / (float)n;
    }
    for (int i = 0; i < SAMPLE_COUNT; i++) arr[i] = tmp[i];
}

static void __attribute__((unused))
run_directional_calibration(bldc_motor_t *motor, struct k_sem *foc_sem, float target_speed, float *output_lut) 
{
    static float temp_sum[SAMPLE_COUNT];
    static uint16_t temp_count[SAMPLE_COUNT];

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        temp_sum[i] = 0.0f;
        temp_count[i] = 0;
    }

    /* Clear any integrator wind-up carried over from the previous pass so the
     * motor doesn't lurch on start (especially when reversing direction). */
    pid_controller_reset(&motor->pid_velocity);

    /* Pre-load the velocity-PID integrator with a small break-away current in the
     * direction of travel. This gives the rotor immediate torque to overcome
     * static friction/cogging WITHOUT stepping the velocity set-point. The old
     * 30% start_frac was a hard velocity step that made the motor lurch at the
     * start of every spin-up; now the set-point ramps smoothly from 0 while this
     * stored current does the break-away. The slew limiter eases it in over a
     * few ms, so there is no snap. */
    motor->pid_velocity.integral_prev = copysignf(CALIB_BREAKAWAY_A, target_speed);

    uint32_t stabilize_ticks = (uint32_t)(2.0f * 5000.0f); 
    uint32_t ramp_ticks = (uint32_t)(1.5f * 5000.0f); /* jerk-free soft-start accel over 1.5 s */
    int move_divider = 0;
    
    for(uint32_t i = 0; i < stabilize_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 5) { // 1000 Hz update for motion control
            move_divider = 0;
            /* Velocity set-point ramps from 0 to full speed with a QUINTIC
             * (5th-order) S-curve. Unlike cubic smoothstep, the quintic has zero
             * acceleration AND zero jerk at both ends, so the demanded torque
             * never changes abruptly anywhere along the ramp - the smoothest
             * possible spin-up. Break-away is handled by the pre-loaded
             * integrator above, so we start cleanly from 0 with no step. */
            float s = target_speed;
            if (i < ramp_ticks) {
                float t = (float)i / (float)ramp_ticks;                        /* 0 -> 1 */
                float smooth = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);   /* smootherstep */
                s = target_speed * smooth;
            }
            bldc_motor_move(motor, s);
        }
        
        float ff = anticogging_get_feedforward(sensor_get_angle(motor->sensor));
        float original_sp = motor->current_sp;
        motor->current_sp += ff; 
        bldc_motor_loop_foc(motor);
        motor->current_sp = original_sp; 
    }

    float time_needed_s = (CALIB_REVOLUTIONS * TWO_PI) / fabs(target_speed);
    uint32_t calib_ticks = (uint32_t)(time_needed_s * 5000.0f);

    for(uint32_t i = 0; i < calib_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 5) { // 1000 Hz update for motion control
            move_divider = 0;
            bldc_motor_move(motor, target_speed);
        }

        float angle = sensor_get_angle(motor->sensor);
        float ff = anticogging_get_feedforward(angle);
        
        float original_sp = motor->current_sp;
        motor->current_sp += ff; 
        bldc_motor_loop_foc(motor);
        motor->current_sp = original_sp; 

        float norm_angle = fmodf(angle, TWO_PI);
        if (norm_angle < 0.0f) norm_angle += TWO_PI;
        int index = (int)((norm_angle / TWO_PI) * SAMPLE_COUNT);
        if (index < 0) index = 0;
        if (index >= SAMPLE_COUNT) index = SAMPLE_COUNT - 1;

        temp_sum[index] += original_sp;
        temp_count[index]++;
    }

    bldc_motor_move(motor, 0.0f);
    bldc_motor_loop_foc(motor);

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (temp_count[i] > 0) {
            output_lut[i] = temp_sum[i] / (float)temp_count[i];
        } else {
            output_lut[i] = (i > 0) ? output_lut[i - 1] : 0.0f;
        }
    }
}

#if ANTICOGGING_STATIC_CALIB && ANTICOGGING_USE_ILC
#define PI_F 3.14159265f

static inline int angle_to_index(float angle)
{
    float na = fmodf(angle, TWO_PI);
    if (na < 0.0f) na += TWO_PI;
    int idx = (int)((na / TWO_PI) * SAMPLE_COUNT);
    if (idx < 0) idx = 0;
    if (idx >= SAMPLE_COUNT) idx = SAMPLE_COUNT - 1;
    return idx;
}

/* One standstill sweep in direction `dir`, applying `ff_lut` as feed-forward and
 * recording the commanded holding current (current_sp) per bin into out_meas. */
static void run_ilc_hold_sweep(bldc_motor_t *motor, struct k_sem *foc_sem,
                               int dir, const float *ff_lut, float *out_meas)
{
    static float    sum_bin[SAMPLE_COUNT];
    static uint16_t cnt_bin[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) { sum_bin[i] = 0.0f; cnt_bin[i] = 0; }

    uint32_t settle_ticks  = (uint32_t)(STATIC_SETTLE_MS  * 5);
    uint32_t measure_ticks = (uint32_t)(STATIC_MEASURE_MS * 5);
    const uint32_t pos_div = (uint32_t)(STATIC_FOC_HZ / STATIC_POS_HZ); /* 10 @ 5 kHz / 500 Hz */
    const float    pos_ts  = (float)pos_div / STATIC_FOC_HZ;

    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);

    sensor_update(motor->sensor);
    float start_angle   = sensor_get_angle(motor->sensor);
    float target_prev   = start_angle;
    float a_prev        = start_angle;
    float vel_f         = 0.0f;
    float last_meas_cur = 0.0f;
    motor->current_sp   = 0.0f;

    for (int p = 0; p < STATIC_POINTS; p++) {
        float frac   = (float)p / (float)STATIC_POINTS;
        float target = start_angle + (float)dir * frac * TWO_PI;

        float integ     = last_meas_cur / STATIC_POS_KI;
        float point_sum = 0.0f;
        int   point_cnt = 0;
        uint32_t pos_div_cnt = pos_div; /* run the position law on the first FOC tick */

        for (uint32_t t = 0; t < settle_ticks + measure_ticks; t++) {
            k_sem_take(foc_sem, K_FOREVER);
            sensor_update(motor->sensor);
            float a = sensor_get_angle(motor->sensor);

            /* Position/damping loop at SENSOR rate. */
            if (++pos_div_cnt >= pos_div) {
                pos_div_cnt = 0;

                float tgt = target;
                if (t < settle_ticks) {
                    float r = (float)t / (float)settle_ticks;
                    r = r * r * (3.0f - 2.0f * r);
                    tgt = target_prev + (target - target_prev) * r;
                }

                float err = tgt - a;
                while (err >  PI_F) err -= TWO_PI;
                while (err < -PI_F) err += TWO_PI;

                float da = a - a_prev;
                while (da >  PI_F) da -= TWO_PI;
                while (da < -PI_F) da += TWO_PI;
                a_prev = a;
                vel_f += STATIC_VEL_ALPHA * (da / pos_ts - vel_f);

                /* Integrátor nabíhá vždy, dokud nedosáhneme cíle */
                integ += err * pos_ts;
                
                float integ_lim = motor->current_limit / STATIC_POS_KI;
                if (integ >  integ_lim) integ =  integ_lim;
                if (integ < -integ_lim) integ = -integ_lim;

                float ff  = (ff_lut && t >= settle_ticks) ? ff_lut[angle_to_index(a)] : 0.0f;
                
                /* OPRAVA: Normální, KLADNÝ výpočet. */
                float cur = STATIC_POS_KP * err + STATIC_POS_KI * integ
                            - STATIC_POS_KD * vel_f + ff;
                            
                if (cur >  motor->current_limit) cur =  motor->current_limit;
                if (cur < -motor->current_limit) cur = -motor->current_limit;
                
                /* OPRAVA: Zpět na normální (KLADNÉ) znaménko */
                motor->current_sp = cur;
            }

            bldc_motor_loop_foc(motor);

            if (t >= settle_ticks) {
                int idx = angle_to_index(a);
                /* Čteme skutečně vyslané Ampéry */
                float meas = motor->current_sp;
                sum_bin[idx] += meas;
                cnt_bin[idx]++;
                point_sum += meas;
                point_cnt++;
            }
        }

        if (point_cnt > 0) last_meas_cur = point_sum / (float)point_cnt;
        target_prev = target;
    }

    motor->current_sp = 0.0f;
    bldc_motor_loop_foc(motor);

    float last = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (cnt_bin[i] > 0) { out_meas[i] = sum_bin[i] / (float)cnt_bin[i]; last = out_meas[i]; }
        else out_meas[i] = last;
    }
}

/* Iterative position-hold calibration. Each iteration: sweep fwd+bwd with the
 * current LUT as FF, average (cancels friction), relax the LUT toward it. */
static void anticogging_calibrate_ilc(bldc_motor_t *motor, struct k_sem *foc_sem)
{
    static float meas_fwd[SAMPLE_COUNT];
    static float meas_bwd[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = 0.0f;

    motion_control_type_t s_ctrl = motor->controller;
    torque_control_type_t s_torque = motor->torque_controller;
    float s_cl = motor->current_limit;

    motor->controller        = TORQUE;
    motor->torque_controller = FOC_CURRENT;
    motor->current_limit     = STATIC_HOLD_CURRENT_LIMIT;

    printk("---> ITERATIVE POSITION-HOLD CALIBRATION START (%d iters, %d points)\n",
           ILC_ITERATIONS, STATIC_POINTS);

    for (int iter = 1; iter <= ILC_ITERATIONS; iter++) {
        run_ilc_hold_sweep(motor, foc_sem, +1, final_lut, meas_fwd);
        k_msleep(300);
        run_ilc_hold_sweep(motor, foc_sem, -1, final_lut, meas_bwd);
        k_msleep(300);

        /* Relax the LUT toward the measured holding current (friction cancelled). */
        float resid = 0.0f;
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float c = 0.5f * (meas_fwd[i] + meas_bwd[i]);
            if (c >  0.5f) c =  0.5f;
            if (c < -0.5f) c = -0.5f;
            resid += fabsf(c - final_lut[i]);
            final_lut[i] += ILC_RELAX_GAIN * (c - final_lut[i]);
        }
        smooth_lut_circular(final_lut, 5);
        printk("---> ILC iter %d/%d  mean|correction|=%.5f A\n",
               iter, ILC_ITERATIONS, (double)(resid / (float)SAMPLE_COUNT));
    }

    /* Remove DC bias so gain > 1 cannot cause self-rotation. */
    float mean = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) mean += final_lut[i];
    mean /= (float)SAMPLE_COUNT;
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] -= mean;

    motor->current_sp = 0.0f;
    bldc_motor_loop_foc(motor);
    motor->controller        = s_ctrl;
    motor->torque_controller = s_torque;
    motor->current_limit     = s_cl;

    printk("---> ILC DONE (DC bias removed: %.5f)\n", (double)mean);
    printk("=== START LUT DUMP ===\nIndex,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) printk("%d,%f\n", i, (double)final_lut[i]);
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
}
#endif /* ANTICOGGING_STATIC_CALIB && ANTICOGGING_USE_ILC */

#if ANTICOGGING_STATIC_CALIB && !ANTICOGGING_USE_ILC
/* One standstill sweep: approach STATIC_POINTS angles in `dir` and record the
 * steady holding current (= detent + direction-dependent friction) per bin. A
 * PI(+D) position loop reaches every target without stick-slip; at rest the
 * commanded current equals the disturbance, i.e. the detent current. */
static void run_static_detent_sweep(bldc_motor_t *motor, struct k_sem *foc_sem,
                                    int dir, float *out_lut)
{
    static float sum_bin[SAMPLE_COUNT];
    static uint16_t cnt_bin[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) { sum_bin[i] = 0.0f; cnt_bin[i] = 0; }

    const float PI_F = 3.14159265f;
    uint32_t settle_ticks  = (uint32_t)(STATIC_SETTLE_MS  * 5); /* 5 ticks/ms @5kHz */
    uint32_t measure_ticks = (uint32_t)(STATIC_MEASURE_MS * 5);
    const uint32_t pos_div = (uint32_t)(STATIC_FOC_HZ / STATIC_POS_HZ);
    const float    pos_ts  = (float)pos_div / STATIC_FOC_HZ;

    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);

    sensor_update(motor->sensor);
    float start_angle   = sensor_get_angle(motor->sensor);
    float target_prev   = start_angle;
    float a_prev        = start_angle;
    float vel_f         = 0.0f;
    float last_meas_cur = 0.0f;  /* warm-start for next point's integrator */
    motor->current_sp   = 0.0f;

    for (int p = 0; p < STATIC_POINTS; p++) {
        float frac   = (float)p / (float)STATIC_POINTS;
        float target = start_angle + (float)dir * frac * TWO_PI;

        /* Reset integrator each point; warm-start from last holding current
         * avoids a large initial transient without cross-point wind-up. */
        float integ     = last_meas_cur / STATIC_POS_KI;
        float point_sum = 0.0f;
        int   point_cnt = 0;
        uint32_t pos_div_cnt = pos_div;

        for (uint32_t t = 0; t < settle_ticks + measure_ticks; t++) {
            k_sem_take(foc_sem, K_FOREVER);
            sensor_update(motor->sensor);
            float a = sensor_get_angle(motor->sensor);

            if (++pos_div_cnt >= pos_div) {
                pos_div_cnt = 0;

                /* Ramp the setpoint from the previous point (smoothstep) so the rotor
                 * creeps over the detent instead of snapping between discrete steps;
                 * hold fixed during the measure window for a true standstill reading. */
                float tgt = target;
                if (t < settle_ticks) {
                    float r = (float)t / (float)settle_ticks;
                    r = r * r * (3.0f - 2.0f * r);
                    tgt = target_prev + (target - target_prev) * r;
                }

                float err = tgt - a;
                while (err >  PI_F) err -= TWO_PI;
                while (err < -PI_F) err += TWO_PI;

                float da = a - a_prev;
                while (da >  PI_F) da -= TWO_PI;
                while (da < -PI_F) da += TWO_PI;
                a_prev = a;
                vel_f += STATIC_VEL_ALPHA * (da / pos_ts - vel_f);

                /* Integrátor nabíhá vždy, dokud nedosáhneme cíle */
                integ += err * pos_ts;
                
                float integ_lim = motor->current_limit / STATIC_POS_KI;
                if (integ >  integ_lim) integ =  integ_lim;
                if (integ < -integ_lim) integ = -integ_lim;

                /* OPRAVA: Normální, KLADNÝ výpočet. */
                float cur = STATIC_POS_KP * err + STATIC_POS_KI * integ
                            - STATIC_POS_KD * vel_f;
                if (cur >  motor->current_limit) cur =  motor->current_limit;
                if (cur < -motor->current_limit) cur = -motor->current_limit;
                
                /* OPRAVA: Zpět na normální (KLADNÉ) znaménko */
                motor->current_sp = cur;
            }

            bldc_motor_loop_foc(motor);

            if (t >= settle_ticks) {                  /* rotor settled: measure */
                float na = fmodf(a, TWO_PI);
                if (na < 0.0f) na += TWO_PI;
                int idx = (int)((na / TWO_PI) * SAMPLE_COUNT);
                if (idx < 0) idx = 0;
                if (idx >= SAMPLE_COUNT) idx = SAMPLE_COUNT - 1;
                /* Čteme skutečně vyslané Ampéry */
                float meas = motor->current_sp;
                sum_bin[idx] += meas;
                cnt_bin[idx]++;
                point_sum += meas;
                point_cnt++;
            }
        }

        if (point_cnt > 0) last_meas_cur = point_sum / (float)point_cnt;
        target_prev = target;
    }

    motor->current_sp = 0.0f;
    bldc_motor_loop_foc(motor);

    float last = 0.0f;                                /* carry-forward gap fill */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (cnt_bin[i] > 0) { out_lut[i] = sum_bin[i] / (float)cnt_bin[i]; last = out_lut[i]; }
        else out_lut[i] = last;
    }
}

/* Bidirectional standstill detent calibration. Averaging the two directions
 * cancels friction and leaves the pure position-dependent detent. Dumps the LUT
 * to serial for hardcoding into anticogging_data.c. */
static void anticogging_calibrate_static_detent(bldc_motor_t *motor, struct k_sem *foc_sem)
{
    static float lut_fwd[SAMPLE_COUNT];
    static float lut_bwd[SAMPLE_COUNT];

    motion_control_type_t saved_ctrl = motor->controller;
    torque_control_type_t saved_torque = motor->torque_controller;
    float saved_c_limit = motor->current_limit;
    motor->controller = TORQUE;
    motor->torque_controller = FOC_CURRENT;
    motor->current_limit = STATIC_HOLD_CURRENT_LIMIT;

    printk("---> STATIC DETENT CALIBRATION START (%d points)\n", STATIC_POINTS);

    run_static_detent_sweep(motor, foc_sem, +1, lut_fwd);
    k_msleep(300);
    run_static_detent_sweep(motor, foc_sem, -1, lut_bwd);

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float v = 0.5f * (lut_fwd[i] + lut_bwd[i]);
        if (v >  0.5f) v =  0.5f;
        if (v < -0.5f) v = -0.5f;
        final_lut[i] = v;
    }
    smooth_lut_circular(final_lut, 5);

    /* Any DC bias creates net torque Ôćĺ self-rotation at gain > 1. Remove it. */
    float lut_mean = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) lut_mean += final_lut[i];
    lut_mean /= (float)SAMPLE_COUNT;
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] -= lut_mean;

    motor->current_sp = 0.0f;
    bldc_motor_loop_foc(motor);
    motor->controller = saved_ctrl;
    motor->torque_controller = saved_torque;
    motor->current_limit = saved_c_limit;

    printk("---> STATIC DETENT CALIBRATION DONE\n");
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
}
#endif /* ANTICOGGING_STATIC_CALIB && !ANTICOGGING_USE_ILC */

/* ================= Persistent LUT storage (raw NVS in settings_storage) =========
 * The calibrated map is kept in a dedicated flash partition so it survives
 * reboots and reflashes without hardcoding it into the firmware image. */
#define ANTICOG_NVS_LUT_ID 1
#define ANTICOG_NVS_ZERO_ANGLE_ID 2

static struct nvs_fs anticog_fs;
static bool anticog_fs_ready;

static int anticog_flash_init(void)
{
    if (anticog_fs_ready) return 0;

    anticog_fs.flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    if (!device_is_ready(anticog_fs.flash_device)) {
        printk("ANTICOG: flash device not ready\n");
        return -ENODEV;
    }

    struct flash_pages_info info;
    int rc = flash_get_page_info_by_offs(anticog_fs.flash_device,
                                         PM_SETTINGS_STORAGE_ADDRESS, &info);
    if (rc) {
        printk("ANTICOG: flash page info error %d\n", rc);
        return rc;
    }

    anticog_fs.offset       = PM_SETTINGS_STORAGE_ADDRESS;
    anticog_fs.sector_size  = info.size;
    anticog_fs.sector_count = PM_SETTINGS_STORAGE_SIZE / info.size;

    rc = nvs_mount(&anticog_fs);
    if (rc) {
        printk("ANTICOG: nvs_mount error %d\n", rc);
        return rc;
    }
    anticog_fs_ready = true;
    return 0;
}

static int anticog_flash_save(const float *lut)
{
    int rc = anticog_flash_init();
    if (rc) return rc;

    ssize_t w = nvs_write(&anticog_fs, ANTICOG_NVS_LUT_ID,
                          lut, SAMPLE_COUNT * sizeof(float));
    if (w < 0) {
        printk("ANTICOG: nvs_write error %d\n", (int)w);
        return (int)w;
    }
    return 0;
}

static int anticog_flash_load(float *lut)
{
    int rc = anticog_flash_init();
    if (rc) return rc;

    ssize_t r = nvs_read(&anticog_fs, ANTICOG_NVS_LUT_ID,
                         lut, SAMPLE_COUNT * sizeof(float));
    if (r != (ssize_t)(SAMPLE_COUNT * sizeof(float))) {
        return -ENOENT;
    }
    return 0;
}

int anticogging_flash_save_zero_angle(float angle)
{
    int rc = anticog_flash_init();
    if (rc) return rc;

    ssize_t w = nvs_write(&anticog_fs, ANTICOG_NVS_ZERO_ANGLE_ID,
                          &angle, sizeof(angle));
    if (w < 0) {
        printk("ANTICOG: zero-angle nvs_write error %d\n", (int)w);
        return (int)w;
    }
    return 0;
}

int anticogging_flash_load_zero_angle(float *angle)
{
    int rc = anticog_flash_init();
    if (rc) return rc;

    ssize_t r = nvs_read(&anticog_fs, ANTICOG_NVS_ZERO_ANGLE_ID,
                         angle, sizeof(*angle));
    if (r != (ssize_t)sizeof(*angle)) {
        return -ENOENT;
    }
    return 0;
}

void anticogging_calibrate(bldc_motor_t *motor, struct k_sem *foc_sem) 
{
#if ANTICOGGING_STATIC_CALIB
    if (motor->motor_status != MOTOR_READY) {
        printk("!!! ANTICOG CALIB BLOCKED: motor not ready !!!\n");
        return;
    }
#if ANTICOGGING_USE_ILC
    anticogging_calibrate_ilc(motor, foc_sem);             /* fills final_lut + dumps */
#else
    anticogging_calibrate_static_detent(motor, foc_sem);   /* fills final_lut + dumps */
#endif

    if (anticog_flash_save(final_lut) == 0) {
        printk("ANTICOG: calibration STORED to flash.\n");
    } else {
        printk("!!! ANTICOG: FAILED to store calibration to flash !!!\n");
    }
    return;
#else
    /* Calibration disabled -> the map MUST be loaded from flash. */
    if (anticog_flash_load(final_lut) != 0) {
        printk("===================================================\n");
        printk("!!! ANTICOG ERROR: no LUT in flash !!!\n");
        printk("Run calibration once: set ANTICOGGING_STATIC_CALIB 1, flash,\n");
        printk("let it calibrate, then set it back to 0. Compensation DISABLED.\n");
        printk("===================================================\n");
        for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = 0.0f;
        return;
    }

    bool nonzero = false;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (final_lut[i] != 0.0f) { nonzero = true; break; }
    }
    if (!nonzero) {
        printk("!!! ANTICOG ERROR: flash LUT is all-zero - run calibration. "
               "Compensation DISABLED.\n");
        return;
    }

    printk("ANTICOG: LUT loaded from flash.\n");
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
    return;
#endif

    /* -------- legacy velocity-ILC calibration (superseded by flash, disabled) --- */
#if 0
    // Check if the LUT has been preloaded with data, if so, skip calibration and use the preloaded data
    bool table_has_data = false;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (anticogging_lut[i] != 0.0f) {
            table_has_data = true;
            break; 
        }
    }

    if(table_has_data)
    {
        printk("LUT has been preloaded with data, skipping calibration.\n");
        printk("=== START LUT DUMP ===\n");
        printk("Index,Current_A\n");

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            final_lut[i] = anticogging_lut[i];
            printk("%d,%f\n", i, (double)final_lut[i]);
        }

        printk("=== END LUT DUMP ===\n");
        k_msleep(1000);
        return;
    }

    /* 1. BEZPEČNOSTNÍ POJISTKA: Kontrola FOC Inicializace */
    if (motor->motor_status != MOTOR_READY) {
        printk("===================================================\n");
        printk("!!! ANTICOGGING CALIBRATION BLOCKED !!!\n");
        printk("FOC initialization (PP check) failed at startup.\n");
        printk("Motor is likely physically blocked or disconnected.\n");
        printk("Please free the motor shaft and restart the board.\n");
        printk("===================================================\n");
        return; 
    }

    static float lut_cw[SAMPLE_COUNT];
    static float lut_ccw[SAMPLE_COUNT];
    static float residual[SAMPLE_COUNT];
    static float accum[SAMPLE_COUNT];

    float saved_kp = motor->pid_velocity.p;
    float saved_ki = motor->pid_velocity.i;
    float saved_v_limit = motor->pid_velocity.limit;
    float saved_c_limit = motor->current_limit;
    float saved_vel_ts = motor->lpf_velocity.Ts;
    float saved_out_ramp = motor->pid_velocity.output_ramp;
    motion_control_type_t saved_controller = motor->controller;

    // set VELOCITY control mode for calibration
    motor->controller = VELOCITY;

    /* OPRAVA: Návrat k "měkké gumě" bez Integrator Windup efektu */
    motor->pid_velocity.p = 0.05f;       /* Hladké obkreslení zubů (žádné kmitání) */
    motor->pid_velocity.i = 0.02f;       /* Miniaturní I-složka jen proti zastavení na tření */
    motor->pid_velocity.limit = 1.0f;    
    motor->current_limit = 1.0f;     

    /* Slew-limit the velocity-PID output (= the current set-point). Without this,
     * when the rotor briefly sticks on a cogging tooth the integrator dumps almost
     * the full current in ~1 ms, the rotor snaps forward, overshoots and the loop
     * rings - exactly the "oscillates in place then unsticks" behaviour. Limiting
     * the current slew to ~20 A/s removes the violent snap while staying far faster
     * than needed to track the cogging ripple (~3 A/s at this speed). Restored
     * after calibration. */
    motor->pid_velocity.output_ramp = 20.0f;

    /* CRITICAL: move() below runs at 1000 Hz (every 5 FOC ticks), but the runtime
     * config hard-codes lpf_velocity.Ts = 0.01 (100 Hz). That 10x mismatch leaves
     * the velocity estimate under-filtered and noisy - a primary cause of the
     * random oscillations. Match Ts to the real update rate so the tf=0.015 s
     * smoothing behaves as intended. Restored after calibration. */
    motor->lpf_velocity.Ts = 0.001f;

    /* Zero the shipped map AND the averaging accumulator. final_lut stays 0 for
     * the WHOLE calibration, so every pass measures the same raw (uncompensated)
     * cogging - the passes are independent estimates of one fixed quantity. */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        final_lut[i] = 0.0f;
        accum[i] = 0.0f;
    }

    printk("---> ANTICOGGING CALIBRATION START (%d averages)\n", CALIB_AVERAGES);

    for (int pass = 1; pass <= CALIB_AVERAGES; pass++) {
        printk("---> PASS %d / %d ---\n", pass, CALIB_AVERAGES);

        run_directional_calibration(motor, foc_sem, CALIB_SPEED_RAD_S, lut_cw);
        k_msleep(500); 
        run_directional_calibration(motor, foc_sem, -CALIB_SPEED_RAD_S, lut_ccw);

        float max_residual_error = 0.0f;
        float sum_residual_error = 0.0f;

        /* Combine both directions (cancels direction-dependent friction, keeps
         * position-dependent cogging), clamp obvious spikes, then spatially
         * smooth this pass BEFORE adding it to the ensemble average. */
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float r = 0.5f * (lut_cw[i] + lut_ccw[i]);
            if (r >  0.5f) r =  0.5f;
            if (r < -0.5f) r = -0.5f;
            residual[i] = r;
        }
        smooth_lut_circular(residual, 5);

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            accum[i] += residual[i];

            float abs_err = fabs(residual[i]);
            sum_residual_error += abs_err;
            if (abs_err > max_residual_error) max_residual_error = abs_err;
        }

        float mean_error = sum_residual_error / (float)SAMPLE_COUNT;
        printk("     Pass mean cogging = %.4f A (Max spike = %.4f A)\n", (double)mean_error, (double)max_residual_error);

        if (pass < CALIB_AVERAGES) k_msleep(500); 
    }

    /* Ensemble average of the independent passes. Because final_lut was never fed
     * back, each pass is an unbiased estimate of the true cogging; the mean keeps
     * that accuracy but with ~sqrt(N) less random noise -> more repeatable map. */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        final_lut[i] = accum[i] * RELAXATION_GAIN / (float)CALIB_AVERAGES;
    }

    /* =========================================================================
     * 3. FILTRACE: závěrečné lehké vyhlazení plně sestavené finální mapy
     * ========================================================================= */
    smooth_lut_circular(final_lut, 3);

    /* Návrat uživatelských PID hodnot po úspěšné kalibraci */
    motor->pid_velocity.p = saved_kp;
    motor->pid_velocity.i = saved_ki;
    motor->pid_velocity.limit = saved_v_limit;
    motor->current_limit = saved_c_limit;
    motor->lpf_velocity.Ts = saved_vel_ts;
    motor->pid_velocity.output_ramp = saved_out_ramp;
    motor->controller = saved_controller;

    printk("---> CALIBRATION COMPLETED ---\n");
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
#endif /* legacy velocity-ILC (disabled) */
}