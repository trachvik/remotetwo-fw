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
extern void sensor_set_observer_bw(float fn_hz);
extern float sensor_get_observer_bw(void);

LOG_MODULE_REGISTER(anticogging, LOG_LEVEL_INF);

#define SAMPLE_COUNT 500
#define TWO_PI 6.28318530718f
/* Minimum safe calibration speed determined by the P-term stall criterion:
 *   KP × ω > I_cog_peak    (P-term alone must break any cogging tooth)
 *   0.05 × ω > 0.13 A  →  ω > 2.6 rad/s
 *
 * At ω = 2.0 rad/s: KP×ω = 0.10 A < 0.13 A cogging peak → the rotor stalls
 * at the hardest teeth, then lurches when the integrator winds up → stick-slip,
 * audible vibration, and a nearly-flat LUT from near-zero mean Iq.
 *
 * At ω = π rad/s ≈ 3.14 rad/s: KP×π = 0.157 A > 0.13 A → the P-term breaks
 * every tooth without any integrator wind-up → smooth, continuous rotation.
 * This is the speed used in the original working calibration (commit 271edba). */
#define CALIB_SPEED_RAD_S 3.14159f   /* π rad/s – original working value */
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
#define CALIB_AVERAGES 2
/* Break-away current pre-loaded into the velocity-PID integrator at the start of
 * each pass. Gives the rotor immediate torque to overcome static friction/cogging
 * so the velocity set-point can ramp smoothly from 0 (no start step = no lurch). */
#define CALIB_BREAKAWAY_A  0.08f   /* preloaded integrator current to start moving */

/* ---- Option B: standstill DETENT calibration ----------------------------------
 * Set to 1 to run a one-time standstill calibration that measures the STATIC
 * detent you feel by hand (correct amplitude AND phase, unlike the spinning map)
 * and DUMPS the LUT to serial. Paste that dump into anticogging_data.c, then set
 * this back to 0. */
#define ANTICOGGING_STATIC_CALIB 1
/* Sample-to-sample POSITION HOLD: 500 discrete angles per revolution.
 *
 * FOC_CURRENT at standstill is a current source: it cancels back-EMF, so the
 * cogging spring (J θ'' + Ks θ = Kt I_cmd) is undamped and rings at ~28 Hz.
 * That is the in-place buzz after a quiet Hi-Z rest probe. The sweep therefore
 * applies VOLTAGE (Uq = I_cmd * R, Ud = 0). BEMF through R damps the spring.
 * The slow PI still walks I_cmd until the shaft sits at each 0.72° target. */
#define ANTICOGGING_POSITION_HOLD 1
#define STATIC_POINTS      500     /* one hold target per LUT bin (full coverage) */
#define STATIC_RAMP_MS     200     /* unused by ILC event-steps; kept for !ILC path */
#define STATIC_SETTLE_MS   200
#define STATIC_MEASURE_MS  80      /* window whose mean I_cmd = detent at arrival */
#define STATIC_FOC_HZ_I    5000    /* integer form, for array sizes */
#define STATIC_FOC_HZ      ((float)STATIC_FOC_HZ_I)
#define STATIC_POS_HZ      500.0f  /* position law; must not exceed the TMAG rate */
#define STEP_TIMEOUT_MS    700     /* give up on a crest and skip to where we are */
#define ARRIVE_RAD         0.008f  /* 8 mrad; inside a 0.72 deg = 12.6 mrad step */

/* Voltage-mode position law. 0.72 deg in a well needs ~Ks*dθ ≈ 42 mA
 * (Ks ≈ 3.3 A/rad). KP*0.0126 rad must supply that without a 100 mA DC
 * crawl: the crawl parked the rotor on the slope (~34 mA) then the PI
 * braked after every tooth jump — sit, sit, hop. Torque is forward-only
 * so a jump does not get pulled back into a well. */
#define POS_KP             3.50f   /* A/rad; 0.72 deg → 44 mA, enough to leave a well */
#define POS_KI             6.00f   /* A/(rad*s); climbs the rest of a tooth if KP stalls */
#define POS_KD             0.0f
#define D_LPF_HZ           8.0f
#define POS_SLEW_A_PER_S   1.0f
#define CRAWL_A            0.02f   /* static friction only; 100 mA caused 16 deg hops */

/* Hard ceiling on the holding current. Also clamped at runtime to the motor's
 * own current_limit, which the locked-rotor test lowers to whatever keeps both
 * low-side shunts inside their sampling window - commanding more than that
 * returns corrupted measurements. */
#define STATIC_HOLD_CURRENT_LIMIT 0.25f

/* ---- Plant gain B = Kt/J [rad/s^2 per A] ---------------------------------------
 * The only plant parameter the servo needs; Kt and J are never needed apart.
 *
 * B is a CONFIGURED constant here, and the pulse test only cross-checks it.
 * That is deliberate, for two reasons.
 *
 * First, the pulse test is fragile: it needs the rotor to break out of a detent
 * within a few milliseconds, and on a boot where it did not (3.7 mrad of travel
 * one way against 31.2 the other) B landed 4x low and the derived gains 4x
 * HIGH - worse than not measuring at all. It is now believed only when it
 * corroborates the constant.
 *
 * Second, B cannot be derived from the nameplate here without picking a Kv
 * convention. This codebase follows SimpleFOC and computes back-EMF as
 * omega/(Kv*sqrt(3))/RPM_TO_RADS, i.e. Kt = 60/(2*pi*Kv*sqrt(3)) = 39 mNm/A at
 * Kv = 141, whereas the plain line-to-line reading is 67.7 mNm/A. Those differ
 * by 1.73x and would scale every gain by that much, and the rotor inertia is not
 * known independently well enough to break the tie (J = Kt/B lands at either
 * 2.7 or 4.6 uKgm^2, both plausible for a 35 mm gimbal rotor). Since only the
 * RATIO Kt/J enters the control law, the measured ratio is the honest input and
 * the ambiguous factorisation is simply not needed. */
#define PLANT_B_NOMINAL    14600.0f /* rad/s^2/A; from the clean pulse measurement */
#define PLANT_PULSE_A      0.15f   /* starting pulse amplitude; raised on retry */
#define PLANT_PULSE_MS     30      /* cap; the velocity limit usually ends it sooner */
#define PLANT_REST_MS      400     /* let the detent catch the rotor between pulses */
#define PLANT_OMEGA_MAX    5.0f    /* rad/s; above this back-EMF eats the Vq budget */
#define PLANT_TRIES        0
#define PLANT_MAX_TICKS    (PLANT_PULSE_MS * STATIC_FOC_HZ_I / 1000)

/* ---- Position-hold stability probe --------------------------------------------
 * Holds one angle and reports how well it actually holds, before committing to a
 * six-minute sweep. If the servo limit-cycles here it limit-cycles at all 500
 * points, and the ringing frequency identifies which term is responsible. */
#define POS_PROBE_MS       800
#define POS_PROBE_TICKS    ((int)(POS_PROBE_MS * STATIC_POS_HZ / 1000.0f))

/* ---- Calibration algorithm selector -------------------------------------------
 * 1 = iterative position-hold (Piccoli & Yim Position-Based method + Qian ILC
 *     relaxation): hold each angle at standstill, inject the current LUT as FF,
 *     measure the filtered holding current, relax the LUT toward it each pass.
 *     Standstill -> no current-loop phase lag -> best for haptics on this HW.
 * 0 = single-pass static detent hold (same physics, one forward+backward sweep). */
#define ANTICOGGING_USE_ILC 1
#define ILC_ITERATIONS     1       /* one bidir pass; stiff ILC pumped 35/176 Hz */
#define ILC_RELAX_GAIN     1.0f    /* lut := measured holding current (bidirectional avg) */
#define STATIC_DT          (1.0f / STATIC_FOC_HZ)

/* Circular smoothing window, in LUT bins. The measured cogging content sits at
 * 22-44 cycles/rev (see the LUT spectrum), i.e. 11-23 bins per cycle at 500
 * bins, so a 5-bin box filter attenuated the 44 cycles/rev component - about a
 * third of the total power - to 47%. 3 bins keeps it at 90%. */
#define LUT_SMOOTH_WINDOW  3

/* Servo gains, filled in by measure_plant_gain() before the sweeps start. */
static float pos_kp = 0.0f;
static float pos_ki = 0.0f;
static float pos_kd = 0.0f;

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
static void
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

#if !ANTICOGGING_POSITION_HOLD
static void
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

    uint32_t stabilize_ticks = (uint32_t)(2.5f * 5000.0f);
    uint32_t ramp_ticks = (uint32_t)(2.0f * 5000.0f);
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

    float time_needed_s = (CALIB_REVOLUTIONS * TWO_PI) / fabsf(target_speed);
    uint32_t calib_ticks = (uint32_t)(time_needed_s * 5000.0f);

    float prog_iq = 0.0f, prog_w = 0.0f, prog_sp = 0.0f;
    int   prog_n = 0;
    float a_mark = sensor_get_angle(motor->sensor);
    float a_unw  = 0.0f;

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

        float da = angle - a_mark;
        while (da >  3.141593f) da -= TWO_PI;
        while (da < -3.141593f) da += TWO_PI;
        a_unw += da;
        a_mark = angle;

        float norm_angle = fmodf(angle, TWO_PI);
        if (norm_angle < 0.0f) norm_angle += TWO_PI;
        int index = (int)((norm_angle / TWO_PI) * SAMPLE_COUNT);
        if (index < 0) index = 0;
        if (index >= SAMPLE_COUNT) index = SAMPLE_COUNT - 1;

        /* Record the measured q current. At this speed the current loop tracks
         * the velocity-PID request; current.q is what actually produces torque. */
        temp_sum[index] += motor->current.q;
        temp_count[index]++;

        prog_iq += motor->current.q;
        prog_sp += original_sp;
        prog_w  += sensor_get_velocity(motor->sensor);
        prog_n++;
        if ((i + 1) % 5000 == 0 && prog_n > 0) {
            float net_w = a_unw / 1.0f; /* unwrapped angle over this 1 s window */
            printk("     spin %+0.2f  t=%ds  net_w=%+.2f  pll_w=%+.2f  "
                   "Iq=%+.0f mA  sp=%+.0f mA  angle=%.0f deg\n",
                   (double)target_speed, (int)((i + 1) / 5000),
                   (double)net_w,
                   (double)(prog_w / (float)prog_n),
                   (double)(prog_iq / (float)prog_n * 1000.0f),
                   (double)(prog_sp / (float)prog_n * 1000.0f),
                   (double)(norm_angle * (360.0f / TWO_PI)));
            prog_iq = 0.0f; prog_w = 0.0f; prog_sp = 0.0f; prog_n = 0;
            a_unw = 0.0f;
        }
    }

    bldc_motor_move(motor, 0.0f);
    bldc_motor_loop_foc(motor);

    /* Come to rest under closed-loop control. Leaving the last PWM on while
     * the caller k_msleeps would keep driving the rotor. */
    for (int t = 0; t < 1500; t++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        if (++move_divider >= 5) {
            move_divider = 0;
            bldc_motor_move(motor, 0.0f);
        }
        bldc_motor_loop_foc(motor);
    }

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (temp_count[i] > 0) {
            output_lut[i] = temp_sum[i] / (float)temp_count[i];
        } else {
            output_lut[i] = (i > 0) ? output_lut[i - 1] : 0.0f;
        }
    }
}
#endif /* !ANTICOGGING_POSITION_HOLD */

/* Shared by both standstill calibration paths (ILC and single-pass detent). */
#if ANTICOGGING_STATIC_CALIB && ANTICOGGING_POSITION_HOLD
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

#if PLANT_TRIES > 0
/* Pulse `dir * amp` from rest and return the implied B, or 0 if the rotor never
 * broke away.
 *
 *     omega(t) = B * integral(i dt)      =>      B = omega_end / integral(i dt)
 *
 * Two things make this the right formulation. Integrating the MEASURED current
 * instead of assuming `amp` from t=0 matters because the current loop needs
 * ~3.6 ms to reach 90%, which is a large fraction of the pulse. And reading the
 * end VELOCITY rather than the displacement avoids integrating twice, so a slow
 * break-away shows up as a small B rather than being hidden.
 *
 * The pulse self-terminates at PLANT_OMEGA_MAX: back-EMF is omega/(Kv*2*pi/60),
 * which at the 913 mV Vq ceiling would consume the whole budget by ~13 rad/s and
 * the current loop would stop delivering what was asked. */
static float plant_pulse_B(bldc_motor_t *motor, struct k_sem *foc_sem,
                           int dir, float amp)
{
    static float theta[PLANT_MAX_TICKS];   /* cumulative unwrapped travel [rad] */
    static float iint[PLANT_MAX_TICKS];    /* cumulative current integral [A*s] */

    const float dt = 1.0f / STATIC_FOC_HZ;
    /* Velocity window: sensor_update is rate limited to 800 us, so 4 ms is the
     * shortest span holding enough independent samples to differentiate. */
    const int win = (int)(0.004f * STATIC_FOC_HZ);

    motor->current_sp = 0.0f;
    for (int t = 0; t < (int)(PLANT_REST_MS * (STATIC_FOC_HZ / 1000.0f)); t++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        bldc_motor_loop_foc(motor);
    }

    sensor_update(motor->sensor);
    float a_prev = sensor_get_angle(motor->sensor);
    float travel = 0.0f, integral = 0.0f;
    int   n_end  = 0;

    motor->current_sp = (float)dir * amp;
    for (int t = 0; t < PLANT_MAX_TICKS; t++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        bldc_motor_loop_foc(motor);

        /* Wrapped increments: the pulse can cross the 0/2*PI seam. */
        float a  = sensor_get_angle(motor->sensor);
        float da = a - a_prev;
        while (da >  PI_F) da -= TWO_PI;
        while (da < -PI_F) da += TWO_PI;
        a_prev = a;

        travel   += (float)dir * da;                     /* sign-corrected */
        integral += (float)dir * motor->current.q * dt;
        theta[t]  = travel;
        iint[t]   = integral;
        n_end     = t;

        if (t >= win) {
            float w = (theta[t] - theta[t - win]) / ((float)win * dt);
            if (w >= PLANT_OMEGA_MAX) break;
        }
    }

    motor->current_sp = 0.0f;
    bldc_motor_loop_foc(motor);

    if (n_end < win) return 0.0f;

    /* The windowed velocity is the average over [n_end-win, n_end], i.e. the
     * instantaneous velocity at the window MIDPOINT - so pair it with the
     * current integral at that same instant, not at n_end. */
    float omega = (theta[n_end] - theta[n_end - win]) / ((float)win * dt);
    float ii    = iint[n_end - win / 2];

    if (!(omega > 0.05f) || !(ii > 1e-6f)) return 0.0f;
    return omega / ii;
}
#endif /* PLANT_TRIES > 0 */

/* Establish B: the configured constant unless the pulse test corroborates it. */
static float resolve_plant_gain(bldc_motor_t *motor, struct k_sem *foc_sem)
{
    const float b_nom = PLANT_B_NOMINAL;
    printk("---> PLANT GAIN: nominal B = %.0f rad/s^2/A\n", (double)b_nom);

#if PLANT_TRIES > 0
    float amp = PLANT_PULSE_A;
    for (int attempt = 1; attempt <= PLANT_TRIES; attempt++) {
        float fwd = plant_pulse_B(motor, foc_sem, +1, amp);
        float bwd = plant_pulse_B(motor, foc_sem, -1, amp);

        printk("---> PLANT GAIN: pulse %.0f mA -> fwd %.0f  bwd %.0f\n",
               (double)(amp * 1000.0f), (double)fwd, (double)bwd);

        if (fwd > 0.0f && bwd > 0.0f) {
            float b_meas = 0.5f * (fwd + bwd);
            float asym   = fabsf(fwd - bwd) / b_meas;
            float ratio  = b_meas / b_nom;

            if (asym < 0.35f && ratio > 0.5f && ratio < 2.0f) {
                printk("---> PLANT GAIN: measured B = %.0f confirms nominal "
                       "(%.0f%% of it, %.0f%% asymmetry) - using measured\n",
                       (double)b_meas, (double)(ratio * 100.0f),
                       (double)(asym * 100.0f));
                return b_meas;
            }
            printk("---> PLANT GAIN: measured B = %.0f rejected "
                   "(%.0f%% of nominal, %.0f%% asymmetry)\n",
                   (double)b_meas, (double)(ratio * 100.0f), (double)(asym * 100.0f));
        } else {
            printk("---> PLANT GAIN: rotor did not break away\n");
        }

        /* Push harder next time, but never past what the shunts can measure. */
        amp *= 1.5f;
        if (amp > motor->current_limit) amp = motor->current_limit;
    }

    printk("---> PLANT GAIN: falling back to nominal B = %.0f\n", (double)b_nom);
#else
    ARG_UNUSED(motor);
    ARG_UNUSED(foc_sem);
#endif
    return b_nom;
}

/* Quasi-static crawl gains. Plant B is logged only. */
static void set_position_gains(float B)
{
    pos_kp = POS_KP;
    pos_ki = POS_KI;
    pos_kd = POS_KD;

    printk("---> POS SERVO: voltage steps  Uq=I*R  B=%.0f  KP=%.2f A/rad  "
           "KI=%.1f A/(rad*s)  slew=%.1f A/s  crawl=%.0f mA  arrive=%.1f deg\n",
           (double)B, (double)pos_kp, (double)pos_ki,
           (double)POS_SLEW_A_PER_S,
           (double)(CRAWL_A * 1000.0f),
           (double)(ARRIVE_RAD * 180.0f / PI_F));
}

static float d_lpf_y = 0.0f;

static void hold_reset_d_lpf(void)
{
    d_lpf_y = 0.0f;
}

/* Forward-only PI + slew. Never torques against `dir`: after a crest jump the
 * old bipolar PI braked to ~30 mA and parked in the next well. */
static float hold_current(bldc_motor_t *motor, float err, float *integ,
                          float pos_ts, float ff, float *last_cmd, int dir)
{
    float vel_raw = sensor_get_velocity(motor->sensor);
    float tf = 1.0f / (TWO_PI * D_LPF_HZ);
    float alpha = pos_ts / (tf + pos_ts);
    d_lpf_y += alpha * (vel_raw - d_lpf_y);

    float fdir = (float)dir;

    if (pos_ki > 1e-6f) {
        /* Integrate only while behind the target. */
        if (err * fdir > 0.0f) {
            *integ += err * pos_ts;
        }
        if (dir > 0 && *integ < 0.0f) *integ = 0.0f;
        if (dir < 0 && *integ > 0.0f) *integ = 0.0f;
        float integ_lim = motor->current_limit / pos_ki;
        if (*integ >  integ_lim) *integ =  integ_lim;
        if (*integ < -integ_lim) *integ = -integ_lim;
    } else {
        *integ = 0.0f;
    }

    float pi = pos_kp * err + pos_ki * (*integ);
    if (dir > 0 && pi < 0.0f) pi = 0.0f;
    if (dir < 0 && pi > 0.0f) pi = 0.0f;

    float iq = pi + ff;
    float max_step = POS_SLEW_A_PER_S * pos_ts;
    float lo = *last_cmd - max_step;
    float hi = *last_cmd + max_step;
    if (iq < lo) iq = lo;
    if (iq > hi) iq = hi;
    *last_cmd = iq;

    float cur = iq - pos_kd * d_lpf_y;
    if (dir > 0 && cur < 0.0f) cur = 0.0f;
    if (dir < 0 && cur > 0.0f) cur = 0.0f;
    if (cur >  motor->current_limit) cur =  motor->current_limit;
    if (cur < -motor->current_limit) cur = -motor->current_limit;
    return cur;
}

/* Map the holding-current command onto a voltage. Current-mode at standstill
 * cancels BEMF and rings the 28 Hz detent spring; a voltage source damps it. */
static void apply_iq_as_voltage(bldc_motor_t *motor, float iq_cmd)
{
    float r = motor->phase_resistance;
    if (!(r > 0.5f) || r > 20.0f) {
        r = 3.4f;
    }
    float uq = iq_cmd * r;
    if (uq >  motor->voltage_limit) uq =  motor->voltage_limit;
    if (uq < -motor->voltage_limit) uq = -motor->voltage_limit;
    motor->voltage.q  = uq;
    motor->voltage.d  = 0.0f;
    motor->current_sp = iq_cmd;
}

static void enable_voltage_drive(bldc_motor_t *motor)
{
    bldc_motor_reset_current_loop_state(motor);
    motor->torque_controller = VOLTAGE;
    motor->voltage.q = 0.0f;
    motor->voltage.d = 0.0f;
    bldc_motor_set_bridge_hiz(motor, false);
    k_busy_wait(20);
}

static void coast_bridge(bldc_motor_t *motor)
{
    motor->voltage.q  = 0.0f;
    motor->voltage.d  = 0.0f;
    motor->current_sp = 0.0f;
    bldc_motor_set_bridge_hiz(motor, true);
}

static float begin_standstill_observer(bldc_motor_t *motor, struct k_sem *foc_sem)
{
    printk("---> standstill: observer %.0f Hz, Hi-Z coast (INL low)\n",
           (double)sensor_get_observer_bw());

    /* current_sp=0 with the bridge ON shorts the windings through SVPWM.
     * Detent spring + inertia + short = 28 Hz ring at ~230 mA. Coast instead
     * so the rotor falls into a well on mechanical damping only. */
    bldc_motor_reset_current_loop_state(motor);
    bldc_motor_set_bridge_hiz(motor, true);

    int idle = (int)(0.50f * STATIC_FOC_HZ);
    for (int t = 0; t < idle; t++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
    }
    return sensor_get_observer_bw();
}

/* Measure standstill while Hi-Z. Do not run the current PI here. */
static bool probe_position_hold(bldc_motor_t *motor, struct k_sem *foc_sem)
{
    const uint32_t pos_div = (uint32_t)(STATIC_FOC_HZ / STATIC_POS_HZ);
    const int      skip    = POS_PROBE_TICKS / 3;

    sensor_update(motor->sensor);
    float target = sensor_get_angle(motor->sensor);

    float err_sum = 0.0f, err_sq = 0.0f, err_min = 1e9f, err_max = -1e9f;
    int   n = 0, crossings = 0;
    float err_prev_ac = 0.0f;

    for (int k = 0; k < POS_PROBE_TICKS; k++) {
        for (uint32_t s = 0; s < pos_div; s++) {
            k_sem_take(foc_sem, K_FOREVER);
            sensor_update(motor->sensor);
        }

        float a   = sensor_get_angle(motor->sensor);
        float err = target - a;
        while (err >  PI_F) err -= TWO_PI;
        while (err < -PI_F) err += TWO_PI;

        if (k >= skip) {
            err_sum += err;
            err_sq  += err * err;
            if (err < err_min) err_min = err;
            if (err > err_max) err_max = err;

            float ac = err - err_sum / (float)(n + 1);
            if ((ac > 0.0f) != (err_prev_ac > 0.0f)) crossings++;
            err_prev_ac = ac;
            n++;
        }
    }

    if (n < 2) {
        return true;
    }

    float mean   = err_sum / (float)n;
    float rms    = sqrtf(err_sq / (float)n - mean * mean);
    float pkpk   = err_max - err_min;
    float span_s = (float)n / STATIC_POS_HZ;
    float freq   = (float)crossings / (2.0f * span_s);

    printk("---> REST PROBE (Hi-Z): err rms %.2f mrad  pk-pk %.2f mrad  bias %.2f mrad\n",
           (double)(rms * 1000.0f), (double)(pkpk * 1000.0f), (double)(mean * 1000.0f));
    printk("---> REST PROBE: ring %.1f Hz (bridge open, Iq not driven)\n",
           (double)freq);

    /* Only refuse if the shaft is free-spinning. Detent tremble is expected. */
    if (pkpk > 1.0f) {
        printk("---> REST PROBE: shaft is spinning - abort.\n");
        return false;
    }
    printk("---> REST PROBE: proceeding to sweep (voltage drive, no current PI).\n");
    return true;
}

#if ANTICOGGING_USE_ILC
/* Event-based 0.72° steps. Wait until the shaft reaches the next bin (or skip
 * if a crest already threw us past it). Record I_cmd at arrival. */
static void run_ilc_hold_sweep(bldc_motor_t *motor, struct k_sem *foc_sem,
                               int dir, const float *ff_lut, float *out_meas)
{
    static float    sum_bin[SAMPLE_COUNT];
    static uint16_t cnt_bin[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) { sum_bin[i] = 0.0f; cnt_bin[i] = 0; }

    uint32_t timeout_ticks = (uint32_t)(STEP_TIMEOUT_MS  * STATIC_FOC_HZ_I / 1000);
    uint32_t measure_ticks = (uint32_t)(STATIC_MEASURE_MS * STATIC_FOC_HZ_I / 1000);
    const uint32_t pos_div = (uint32_t)(STATIC_FOC_HZ / STATIC_POS_HZ);
    const float    pos_ts  = (float)pos_div / STATIC_FOC_HZ;
    const float    fdir    = (float)dir;

    bldc_motor_reset_current_loop_state(motor);
    hold_reset_d_lpf();

    printk("     hold sweep dir %+d  (%d event-steps, %.2f deg/step, Uq=I*R)\n",
           dir, STATIC_POINTS, (double)(360.0f / (float)STATIC_POINTS));

    sensor_update(motor->sensor);
    float start_angle = sensor_get_angle(motor->sensor);
    float last_cmd    = 0.0f;
    float integ       = 0.0f;
    float a_prev      = start_angle;
    float travel_acc  = 0.0f;
    motor->current_sp = 0.0f;

    enable_voltage_drive(motor);

    float prog_err = 0.0f, prog_cur = 0.0f;
    int   prog_n = 0, prog_railed = 0, prog_skip = 0, last_print = 0;

    for (int p = 0; p < STATIC_POINTS; ) {
        float target = start_angle + fdir * (float)p / (float)STATIC_POINTS * TWO_PI;

        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        float a = sensor_get_angle(motor->sensor);
        {
            float da = a - a_prev;
            while (da >  PI_F) da -= TWO_PI;
            while (da < -PI_F) da += TWO_PI;
            travel_acc += da;
            a_prev = a;
        }

        float err = target - a;
        while (err >  PI_F) err -= TWO_PI;
        while (err < -PI_F) err += TWO_PI;

        /* Crest jump already carried us past this bin: skip, do not sit. */
        if (err * fdir < -ARRIVE_RAD) {
            integ = 0.0f;
            last_cmd = 0.0f;
            apply_iq_as_voltage(motor, 0.0f);
            bldc_motor_loop_foc(motor);
            prog_skip++;
            p++;
        } else {
        uint32_t pos_div_cnt = pos_div;
        bool arrived = (fabsf(err) < ARRIVE_RAD);
        for (uint32_t t = 0; t < timeout_ticks && !arrived; t++) {
            if (t > 0) {
                k_sem_take(foc_sem, K_FOREVER);
                sensor_update(motor->sensor);
                a = sensor_get_angle(motor->sensor);
                float da = a - a_prev;
                while (da >  PI_F) da -= TWO_PI;
                while (da < -PI_F) da += TWO_PI;
                travel_acc += da;
                a_prev = a;
            }

            if (++pos_div_cnt >= pos_div) {
                pos_div_cnt = 0;
                err = target - a;
                while (err >  PI_F) err -= TWO_PI;
                while (err < -PI_F) err += TWO_PI;

                if (err * fdir < -ARRIVE_RAD) {
                    integ = 0.0f;
                    apply_iq_as_voltage(motor, 0.0f);
                    arrived = true;
                } else {
                    float ff = (ff_lut ? ff_lut[angle_to_index(a)] : 0.0f)
                               + CRAWL_A * fdir;
                    float cur = hold_current(motor, err, &integ, pos_ts, ff,
                                             &last_cmd, dir);
                    if (cur == motor->current_limit || cur == -motor->current_limit) {
                        prog_railed++;
                    }
                    apply_iq_as_voltage(motor, cur);
                    if (fabsf(err) < ARRIVE_RAD && fabsf(d_lpf_y) < 0.5f) {
                        arrived = true;
                    }
                }
            }
            bldc_motor_loop_foc(motor);
        }

        /* Measure I_cmd at this angle (commanded current = detent estimate). */
        for (uint32_t t = 0; t < measure_ticks; t++) {
            k_sem_take(foc_sem, K_FOREVER);
            sensor_update(motor->sensor);
            a = sensor_get_angle(motor->sensor);
            {
                float da = a - a_prev;
                while (da >  PI_F) da -= TWO_PI;
                while (da < -PI_F) da += TWO_PI;
                travel_acc += da;
                a_prev = a;
            }
            if (++pos_div_cnt >= pos_div) {
                pos_div_cnt = 0;
                err = target - a;
                while (err >  PI_F) err -= TWO_PI;
                while (err < -PI_F) err += TWO_PI;
                float ff = (ff_lut ? ff_lut[angle_to_index(a)] : 0.0f)
                           + CRAWL_A * fdir;
                if (err * fdir < 0.0f) {
                    apply_iq_as_voltage(motor, 0.0f);
                } else {
                    float cur = hold_current(motor, err, &integ, pos_ts, ff,
                                             &last_cmd, dir);
                    apply_iq_as_voltage(motor, cur);
                }
                prog_err += fabsf(err);
                prog_cur += fabsf(motor->current_sp);
                prog_n++;
            }
            bldc_motor_loop_foc(motor);
            int idx = angle_to_index(a);
            sum_bin[idx] += motor->current_sp;
            cnt_bin[idx]++;
        }

        p++;
        } /* else: not skipped */

        if (p >= last_print + 50) {
            printk("     dir %+d  point %d/%d  |err| %.2f mrad  |Iq| %.0f mA  "
                   "travel %.1f deg  skip %d  railed %d\n",
                   dir, p, STATIC_POINTS,
                   prog_n > 0 ? (double)(prog_err / (float)prog_n * 1000.0f) : 0.0,
                   prog_n > 0 ? (double)(prog_cur / (float)prog_n * 1000.0f) : 0.0,
                   (double)(travel_acc * (180.0f / PI_F)),
                   prog_skip, prog_railed);
            prog_err = 0.0f; prog_cur = 0.0f; prog_n = 0;
            prog_railed = 0; prog_skip = 0;
            last_print = p;
        }
    }

    coast_bridge(motor);

    float last = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (cnt_bin[i] > 0) { out_meas[i] = sum_bin[i] / (float)cnt_bin[i]; last = out_meas[i]; }
        else out_meas[i] = last;
    }
}

/* Iterative position-hold calibration. Each iteration: sweep fwd+bwd with the
 * current LUT as FF, average (cancels friction), relax the LUT toward it. */
static int anticogging_calibrate_ilc(bldc_motor_t *motor, struct k_sem *foc_sem)
{
    static float meas_fwd[SAMPLE_COUNT];
    static float meas_bwd[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = 0.0f;

    motion_control_type_t s_ctrl = motor->controller;
    torque_control_type_t s_torque = motor->torque_controller;
    float s_cl = motor->current_limit;

    motor->controller        = TORQUE;
    motor->torque_controller = VOLTAGE;

    /* Never raise the limit above what the caller left in place: the locked-rotor
     * test lowers it to the largest current at which both low-side shunts are
     * still sampled inside their conduction window. */
    motor->current_limit = (s_cl < STATIC_HOLD_CURRENT_LIMIT)
                         ? s_cl : STATIC_HOLD_CURRENT_LIMIT;

    printk("---> ITERATIVE POSITION-HOLD CALIBRATION START (%d iters, %d points, "
           "%.2f deg/step, event-wait %d ms, current limit %.0f mA, voltage drive)\n",
           ILC_ITERATIONS, STATIC_POINTS,
           (double)(360.0f / (float)STATIC_POINTS),
           STEP_TIMEOUT_MS,
           (double)(motor->current_limit * 1000.0f));

    float saved_pll = begin_standstill_observer(motor, foc_sem);
    set_position_gains(resolve_plant_gain(motor, foc_sem));
    if (!probe_position_hold(motor, foc_sem)) {
        coast_bridge(motor);
        motor->controller        = s_ctrl;
        motor->torque_controller = s_torque;
        motor->current_limit     = s_cl;
        sensor_set_observer_bw(saved_pll);
        bldc_motor_set_bridge_hiz(motor, false);
        printk("!!! ANTICOG: calibration aborted (hold unstable). LUT not stored.\n");
        return -1;
    }

    for (int iter = 1; iter <= ILC_ITERATIONS; iter++) {
        run_ilc_hold_sweep(motor, foc_sem, +1, final_lut, meas_fwd);
        k_msleep(300);
        run_ilc_hold_sweep(motor, foc_sem, -1, final_lut, meas_bwd);
        k_msleep(300);

        /* Relax the LUT toward the measured holding current (friction cancelled). */
        float clamp = motor->current_limit;
        float resid = 0.0f;
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float c = 0.5f * (meas_fwd[i] + meas_bwd[i]);
            if (c >  clamp) c =  clamp;
            if (c < -clamp) c = -clamp;
            resid += fabsf(c - final_lut[i]);
            final_lut[i] += ILC_RELAX_GAIN * (c - final_lut[i]);
        }
        smooth_lut_circular(final_lut, LUT_SMOOTH_WINDOW);
        printk("---> ILC iter %d/%d  mean|correction|=%.5f A\n",
               iter, ILC_ITERATIONS, (double)(resid / (float)SAMPLE_COUNT));
    }

    /* Remove DC bias so gain > 1 cannot cause self-rotation. */
    float mean = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) mean += final_lut[i];
    mean /= (float)SAMPLE_COUNT;
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] -= mean;

    motor->current_sp = 0.0f;
    motor->voltage.q  = 0.0f;
    motor->voltage.d  = 0.0f;
    bldc_motor_reset_current_loop_state(motor);
    motor->controller        = s_ctrl;
    motor->torque_controller = s_torque;
    motor->current_limit     = s_cl;
    sensor_set_observer_bw(saved_pll);
    bldc_motor_set_bridge_hiz(motor, false);

    printk("---> ILC DONE (DC bias removed: %.5f)\n", (double)mean);
    printk("=== START LUT DUMP ===\nIndex,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) printk("%d,%f\n", i, (double)final_lut[i]);
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
    return 0;
}
#endif /* ANTICOGGING_USE_ILC */

#if !ANTICOGGING_USE_ILC
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

    uint32_t ramp_ticks    = (uint32_t)(STATIC_RAMP_MS    * STATIC_FOC_HZ_I / 1000);
    uint32_t settle_ticks  = (uint32_t)(STATIC_SETTLE_MS  * STATIC_FOC_HZ_I / 1000);
    uint32_t measure_ticks = (uint32_t)(STATIC_MEASURE_MS * STATIC_FOC_HZ_I / 1000);
    uint32_t total_ticks   = ramp_ticks + settle_ticks + measure_ticks;
    const uint32_t pos_div = (uint32_t)(STATIC_FOC_HZ / STATIC_POS_HZ);
    const float    pos_ts  = (float)pos_div / STATIC_FOC_HZ;

    bldc_motor_reset_current_loop_state(motor);
    hold_reset_d_lpf();
    enable_voltage_drive(motor);

    sensor_update(motor->sensor);
    float start_angle   = sensor_get_angle(motor->sensor);
    float target_prev   = start_angle;
    float last_meas_cur = 0.0f;  /* warm-start for next point's integrator */
    float last_cmd      = 0.0f;
    motor->current_sp   = 0.0f;

    for (int p = 0; p < STATIC_POINTS; p++) {
        float frac   = (float)p / (float)STATIC_POINTS;
        float target = start_angle + (float)dir * frac * TWO_PI;

        /* Reset integrator each point; warm-start from last holding current
         * avoids a large initial transient without cross-point wind-up. */
        float integ     = last_meas_cur / pos_ki;
        float point_sum = 0.0f;
        int   point_cnt = 0;
        uint32_t pos_div_cnt = pos_div;

        for (uint32_t t = 0; t < total_ticks; t++) {
            k_sem_take(foc_sem, K_FOREVER);
            sensor_update(motor->sensor);
            float a = sensor_get_angle(motor->sensor);

            if (++pos_div_cnt >= pos_div) {
                pos_div_cnt = 0;

                /* Smoothstep bump to the next bin; then hold still for settle+measure. */
                float tgt = target;
                if (t < ramp_ticks) {
                    float r = (float)t / (float)ramp_ticks;
                    r = r * r * (3.0f - 2.0f * r);
                    tgt = target_prev + (target - target_prev) * r;
                }

                float err = tgt - a;
                while (err >  PI_F) err -= TWO_PI;
                while (err < -PI_F) err += TWO_PI;

                float cur = hold_current(motor, err, &integ, pos_ts, 0.0f, &last_cmd, dir);
                apply_iq_as_voltage(motor, cur);
            }

            bldc_motor_loop_foc(motor);

            if (t >= ramp_ticks + settle_ticks) {
                float na = fmodf(a, TWO_PI);
                if (na < 0.0f) na += TWO_PI;
                int idx = (int)((na / TWO_PI) * SAMPLE_COUNT);
                if (idx < 0) idx = 0;
                if (idx >= SAMPLE_COUNT) idx = SAMPLE_COUNT - 1;
                float meas = motor->current.q;
                sum_bin[idx] += meas;
                cnt_bin[idx]++;
                point_sum += meas;
                point_cnt++;
            }
        }

        if (point_cnt > 0) last_meas_cur = point_sum / (float)point_cnt;
        target_prev = target;
    }

    coast_bridge(motor);

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
    motor->torque_controller = VOLTAGE;
    motor->current_limit = (saved_c_limit < STATIC_HOLD_CURRENT_LIMIT)
                         ? saved_c_limit : STATIC_HOLD_CURRENT_LIMIT;

    printk("---> STATIC DETENT CALIBRATION START (%d points, voltage drive)\n", STATIC_POINTS);

    float saved_pll = begin_standstill_observer(motor, foc_sem);
    set_position_gains(resolve_plant_gain(motor, foc_sem));
    probe_position_hold(motor, foc_sem);

    run_static_detent_sweep(motor, foc_sem, +1, lut_fwd);
    k_msleep(300);
    run_static_detent_sweep(motor, foc_sem, -1, lut_bwd);

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float v = 0.5f * (lut_fwd[i] + lut_bwd[i]);
        if (v >  motor->current_limit) v =  motor->current_limit;
        if (v < -motor->current_limit) v = -motor->current_limit;
        final_lut[i] = v;
    }
    smooth_lut_circular(final_lut, LUT_SMOOTH_WINDOW);

    /* Any DC bias creates net torque Ôćĺ self-rotation at gain > 1. Remove it. */
    float lut_mean = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) lut_mean += final_lut[i];
    lut_mean /= (float)SAMPLE_COUNT;
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] -= lut_mean;

    motor->current_sp = 0.0f;
    motor->voltage.q  = 0.0f;
    motor->voltage.d  = 0.0f;
    bldc_motor_reset_current_loop_state(motor);
    motor->controller = saved_ctrl;
    motor->torque_controller = saved_torque;
    motor->current_limit = saved_c_limit;
    sensor_set_observer_bw(saved_pll);
    bldc_motor_set_bridge_hiz(motor, false);

    printk("---> STATIC DETENT CALIBRATION DONE\n");
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
}
#endif /* !ANTICOGGING_USE_ILC */
#endif /* ANTICOGGING_STATIC_CALIB && ANTICOGGING_POSITION_HOLD */

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

#if !ANTICOGGING_POSITION_HOLD
/* Slow constant-speed map. Bidirectional average cancels friction; the remaining
 * current vs angle is the detent cogging profile.
 *
 * Note on shaft speed: the sensor magnet is coupled at 0.5x shaft speed, so
 * pll_omega = 0.5 * mechanical_omega. The velocity loop therefore drives the
 * shaft to 2 * CALIB_SPEED_RAD_S (~6.28 rad/s actual). At that speed cogging
 * is at 36 Hz. The observer is temporarily raised to CALIB_PLL_FN_HZ (80 Hz)
 * so velocity ripple from cogging is tracked with only 7% attenuation instead
 * of 73%, giving a LUT that is 93% of the true cogging amplitude. */
static void anticogging_calibrate_spin(bldc_motor_t *motor, struct k_sem *foc_sem)
{
    static float lut_cw[SAMPLE_COUNT];
    static float lut_ccw[SAMPLE_COUNT];
    static float residual[SAMPLE_COUNT];
    static float accum[SAMPLE_COUNT];

    float saved_kp = motor->pid_velocity.p;
    float saved_ki = motor->pid_velocity.i;
    float saved_v_limit = motor->pid_velocity.limit;
    float saved_c_limit = motor->current_limit;
    float saved_vel_ts = motor->lpf_velocity.Ts;
    float saved_vel_tf = motor->lpf_velocity.tf;
    float saved_pid_ts = motor->pid_velocity.Ts;
    float saved_out_ramp = motor->pid_velocity.output_ramp;
    motion_control_type_t saved_controller = motor->controller;
    motor->controller = VELOCITY;
    motor->torque_controller = FOC_CURRENT;

    /* At 2 rad/s inertia carries the rotor through each tooth; the velocity
     * loop only needs to maintain average speed against friction. These are
     * the empirical gains from the original working calibration code. The
     * IMC formula (KP = 2*zeta*wn/B) with B=14600 gives KP=0.0046 at 6 Hz,
     * which produces only 1.8 mA per 0.4 rad/s error – orders of magnitude
     * too small to maintain speed against ~100 mA of friction. */
    motor->pid_velocity.p = 0.05f;
    motor->pid_velocity.i = 0.02f;
    motor->pid_velocity.d = 0.0f;
    motor->pid_velocity.limit = motor->current_limit;
    motor->pid_velocity.output_ramp = 0.0f;
    motor->pid_velocity.Ts = 0.001f;       /* move() runs at 1 kHz */
    /* The 20 Hz PLL attenuates the 36 Hz cogging velocity ripple down to ~27%,
     * but it also cushions the velocity loop against reacting to individual cogging
     * teeth. Without this filter the loop becomes unstable or stalls the motor.
     * tf=0.003 gives ≈53 Hz cutoff; combined with the 20 Hz PLL this keeps the
     * velocity-loop phase margin around +55°. */
    motor->lpf_velocity.Ts = 0.001f;
    motor->lpf_velocity.tf = 0.003f;
    motor->lpf_velocity.y_prev = 0.0f;

    printk("---> SLOW-SPIN CALIBRATION START (%d averages, %.2f rad/s, "
           "vel KP=%.4f KI=%.3f, Ilim=%.0f mA)\n",
           CALIB_AVERAGES, (double)CALIB_SPEED_RAD_S,
           (double)motor->pid_velocity.p, (double)motor->pid_velocity.i,
           (double)(motor->current_limit * 1000.0f));

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        final_lut[i] = 0.0f;
        accum[i] = 0.0f;
    }

    float clamp = motor->current_limit;
    for (int pass = 1; pass <= CALIB_AVERAGES; pass++) {
        printk("---> PASS %d / %d ---\n", pass, CALIB_AVERAGES);

        run_directional_calibration(motor, foc_sem,  CALIB_SPEED_RAD_S, lut_cw);
        k_msleep(400);
        run_directional_calibration(motor, foc_sem, -CALIB_SPEED_RAD_S, lut_ccw);

        float sum_abs = 0.0f, max_abs = 0.0f;
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float r = 0.5f * (lut_cw[i] + lut_ccw[i]);
            if (r >  clamp) r =  clamp;
            if (r < -clamp) r = -clamp;
            residual[i] = r;
            float a = fabsf(r);
            sum_abs += a;
            if (a > max_abs) max_abs = a;
        }
        smooth_lut_circular(residual, 3);

        for (int i = 0; i < SAMPLE_COUNT; i++) accum[i] += residual[i];

        printk("     Pass mean |Iq| = %.4f A  (peak %.4f A)\n",
               (double)(sum_abs / (float)SAMPLE_COUNT), (double)max_abs);
        if (pass < CALIB_AVERAGES) k_msleep(400);
    }

    float mean = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        final_lut[i] = accum[i] / (float)CALIB_AVERAGES;
        mean += final_lut[i];
    }
    mean /= (float)SAMPLE_COUNT;
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] -= mean;
    smooth_lut_circular(final_lut, 3);

    motor->current_sp = 0.0f;
    bldc_motor_move(motor, 0.0f);
    bldc_motor_loop_foc(motor);

    motor->pid_velocity.p = saved_kp;
    motor->pid_velocity.i = saved_ki;
    motor->pid_velocity.limit = saved_v_limit;
    motor->pid_velocity.Ts = saved_pid_ts;
    motor->current_limit = saved_c_limit;
    motor->lpf_velocity.Ts = saved_vel_ts;
    motor->lpf_velocity.tf = saved_vel_tf;
    motor->pid_velocity.output_ramp = saved_out_ramp;
    motor->controller = saved_controller;

    /* Clear all PID state so no integrator wind-up from calibration carries into haptic mode. */
    pid_controller_reset(&motor->pid_velocity);
    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);
    motor->current_sp = 0.0f;

    printk("---> SLOW-SPIN CALIBRATION DONE (DC bias removed: %.5f A)\n", (double)mean);
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
}
#endif /* !ANTICOGGING_POSITION_HOLD */

void anticogging_calibrate(bldc_motor_t *motor, struct k_sem *foc_sem) 
{
#if ANTICOGGING_STATIC_CALIB
    if (motor->motor_status != MOTOR_READY) {
        printk("!!! ANTICOG CALIB BLOCKED: motor not ready !!!\n");
        return;
    }
#if ANTICOGGING_POSITION_HOLD
#if ANTICOGGING_USE_ILC
    if (anticogging_calibrate_ilc(motor, foc_sem) != 0) {
        return;
    }
#else
    anticogging_calibrate_static_detent(motor, foc_sem);
#endif
#else
    anticogging_calibrate_spin(motor, foc_sem);
#endif

    if (anticog_flash_save(final_lut) == 0) {
        printk("ANTICOG: calibration STORED to flash.\n");
    } else {
        printk("!!! ANTICOG: FAILED to store calibration to flash !!!\n");
    }
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
#endif
}