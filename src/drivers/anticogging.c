/*
 * anticogging.c
 * Final Consistent Calibration (Lerp + Single-Pass Filter)
 */

#include "anticogging.h"
#include "drivers/bldc_motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <math.h>

extern float sensor_get_angle(void *sensor);
extern void sensor_update(void *sensor);

LOG_MODULE_REGISTER(anticogging, LOG_LEVEL_INF);

#define SAMPLE_COUNT 500
#define TWO_PI 6.28318530718f
#define CALIB_SPEED_RAD_S 3.14159f
#define CALIB_REVOLUTIONS 3
#define MAX_ITERATIONS 5
#define RELAXATION_GAIN 0.5f
#define ERROR_THRESHOLD 0.005f 

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
static void smooth_lut_circular(float *arr, int window)
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

static void run_directional_calibration(bldc_motor_t *motor, struct k_sem *foc_sem, float target_speed, float *output_lut) 
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

    uint32_t stabilize_ticks = (uint32_t)(2.0f * 5000.0f); 
    uint32_t ramp_ticks = (uint32_t)(1.2f * 5000.0f); /* soft-start accel over 1.2 s */
    int move_divider = 0;
    
    for(uint32_t i = 0; i < stabilize_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 5) { // 1000 Hz update for motion control
            move_divider = 0;
            /* Smooth acceleration into the (uncompensated) warm-up spin. Starting
             * from 0 lets the rotor stall against static friction/cogging and then
             * lurch free; stepping straight to full speed overshoots and rings.
             * Begin at a fraction that gives immediate break-away torque, then use
             * a smoothstep (ease in/out) curve up to full speed. */
            float s = target_speed;
            if (i < ramp_ticks) {
                float t = (float)i / (float)ramp_ticks;      /* 0 -> 1 */
                float smooth = t * t * (3.0f - 2.0f * t);    /* smoothstep */
                float start_frac = 0.30f;                    /* immediate break-away */
                s = target_speed * (start_frac + (1.0f - start_frac) * smooth);
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

void anticogging_calibrate(bldc_motor_t *motor, struct k_sem *foc_sem) 
{
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
    static float best_lut[SAMPLE_COUNT];

    float best_error = 1e30f;
    float prev_error = 1e30f;

    float saved_kp = motor->pid_velocity.p;
    float saved_ki = motor->pid_velocity.i;
    float saved_v_limit = motor->pid_velocity.limit;
    float saved_c_limit = motor->current_limit;
    float saved_vel_ts = motor->lpf_velocity.Ts;
    float saved_out_ramp = motor->pid_velocity.output_ramp;

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

    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = 0.0f;
    
    printk("---> ANTICOGGING CALIBRATION START\n");

    for (int iter = 1; iter <= MAX_ITERATIONS; iter++) {
        printk("---> ITERATION %d / %d ---\n", iter, MAX_ITERATIONS);

        run_directional_calibration(motor, foc_sem, CALIB_SPEED_RAD_S, lut_cw);
        k_msleep(500); 
        run_directional_calibration(motor, foc_sem, -CALIB_SPEED_RAD_S, lut_ccw);

        float max_residual_error = 0.0f;
        float sum_residual_error = 0.0f;

        /* Combine both directions (cancels direction-dependent friction, keeps
         * position-dependent cogging), clamp obvious spikes, then spatially
         * smooth the residual BEFORE feeding it back. A noisy residual is what
         * makes the feedforward - and thus the motion - jerky and oscillate. */
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float r = 0.5f * (lut_cw[i] + lut_ccw[i]);
            if (r >  0.5f) r =  0.5f;
            if (r < -0.5f) r = -0.5f;
            residual[i] = r;
        }
        smooth_lut_circular(residual, 5);

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            final_lut[i] += residual[i] * RELAXATION_GAIN;

            float abs_err = fabs(residual[i]);
            sum_residual_error += abs_err;
            if (abs_err > max_residual_error) max_residual_error = abs_err;
        }

        float mean_error = sum_residual_error / (float)SAMPLE_COUNT;
        printk("     Mean residual error = %.4f A (Max spike = %.4f A)\n", (double)mean_error, (double)max_residual_error);

        /* Keep the BEST map. On this hardware the residual measurement lags the
         * real cogging at CALIB_SPEED_RAD_S (velocity-filter + current-loop lag),
         * so beyond the first pass the error stops dropping and can creep up.
         * Snapshot the lowest-error map and never ship a worse one. */
        if (mean_error < best_error) {
            best_error = mean_error;
            for (int i = 0; i < SAMPLE_COUNT; i++) best_lut[i] = final_lut[i];
        }

        if (max_residual_error < ERROR_THRESHOLD) {
            printk("---> Error below threshold. Calibration finished early!\n");
            break; 
        }
        if (mean_error > prev_error) {
            printk("---> Error increased vs previous iteration - stopping (keeping best).\n");
            break;
        }
        prev_error = mean_error;
        k_msleep(500); 
    }

    /* Ship the best map found across the iterations. */
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = best_lut[i];

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

    printk("---> CALIBRATION COMPLETED ---\n");
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
}
