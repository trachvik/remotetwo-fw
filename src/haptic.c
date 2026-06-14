#include "haptic.h"
#include "drivers/tmag5170_sensor.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <arm_math.h>

LOG_MODULE_REGISTER(haptic, LOG_LEVEL_DBG);

/* Sensor abstraction implemented in tmag5170_sensor.c. */

#define SUPPLY_VOLTAGE 3.75f
#define HAPTIC_OUTPUT_GAIN 2.0f
#define HAPTIC_VOLTAGE_LIMIT SUPPLY_VOLTAGE

/* Voltage-control haptic tuning (loop rate: 1 kHz = K_MSEC(1)).
 * Velocity LPF alpha = 0.1  at 1 kHz -> about 16 Hz bandwidth (detent, fast response).
 * Velocity LPF alpha = 0.01 at 1 kHz -> about 1.6 Hz bandwidth (smooth mode, avoids zero crossing between encoder ticks).
 * At 0.5 rad/s the encoder fires about every 77 ms; alpha=0.01 keeps velocity from decaying to zero between ticks.
 */
#define SMOOTH_KV           0.3f    /* V*s/rad velocity damping in smooth mode */
#define SMOOTH_VEL_ALPHA    0.03f   /* velocity IIR alpha for smooth mode, tau about 33 ms */
#define SMOOTH_VEL_THRESH   0.05f   /* rad/s threshold below which motor freewheels */
#define SMOOTH_NAV_STEP_ANGLE_RAD 0.06f /* fine virtual navigation step in smooth mode */
#define SMOOTH_VDETENT_STEPS 16
#define SMOOTH_VDETENT_STEP_RAD (_2PI / (float)SMOOTH_VDETENT_STEPS)
#define SMOOTH_VDETENT_ARM_VEL 0.12f
#define SMOOTH_VDETENT_ARM_MS 80
#define VCLICK_HOLD_MS 500
#define VCLICK_LONG_HOLD_MS 2000 /* reserved for future mode-select command */
#define GESTURE_ARM_ZONE_MIN   0.14f /* slightly smaller dead zone for Enter/Back evaluation */
#define GESTURE_RELEASE_ZONE   0.09f /* slightly tighter release zone to reset gesture */
#define HAPTIC_ZERO_ZONE_RAD   (0.5f * (_PI / 180.0f))  /* hard zero below 0.5 deg, no PWM output */
#define HAPTIC_BLEND_ZONE_RAD  (2.5f * (_PI / 180.0f))  /* smooth force ramp from 0 to full between 0.5 and 2.5 deg */
#define HAPTIC_DETENT_AMP_V 1.0f    /* peak spring voltage in detent mode */
#define DETENT_KV           0.08f   /* V*s/rad velocity damping in detent mode */
#define VEL_LPF_ALPHA       0.1f    /* fast velocity IIR in detent mode, about 16 Hz BW */
#define SMOOTH_IIR_ALPHA    0.1f    /* output IIR at 1 kHz, tau about 10 ms */
#define DETENT_IIR_ALPHA    0.30f   /* fast response in detent mode, tau about 1.1 ms */
                                     /* was 0.30f, increased for snappier detent response */

/* PWM pin definitions removed, hardware is fully described in the board overlay. */

/* Motor parameters */
#define MOTOR_POLE_PAIRS 11     /* Number of pole pairs */
#define MOTOR_PHASE_RESISTANCE 5.6f  /* Ohms */
#define MOTOR_KV_RATING 320.0f  /* rpm/V */
#define MOTOR_INDUCTANCE 0.0001f /* H */

/* Prefer btn_south, keep compatibility with older overlays and board buttons. */
#if DT_NODE_EXISTS(DT_NODELABEL(btn_south))
#define HAPTIC_BTN_NODE DT_NODELABEL(btn_south)
#elif DT_NODE_EXISTS(DT_NODELABEL(btn_east))
#define HAPTIC_BTN_NODE DT_NODELABEL(btn_east)
#elif DT_NODE_EXISTS(DT_NODELABEL(button_b1))
#define HAPTIC_BTN_NODE DT_NODELABEL(button_b1)
#else
#define HAPTIC_BTN_NODE DT_INVALID_NODE
#endif

static const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET_OR(HAPTIC_BTN_NODE, gpios, {0});

/* Haptic state variables */
static float start_angle = 0.0f;
static int num_steps_old = 0;
static int prev_detent_index = 0;
static bool detent_prev_init = false;
static float cumulative_angle = 0.0f;   /* unwrapped mechanical angle since start [rad] */
static void (*g_step_cb)(int dir) = NULL;
static void (*g_virtual_click_cb)(int dir) = NULL;
static int g_num_steps_current = 0;        /* current detent count, exposed via getter */
static int g_num_steps_override = -1;      /* -1 = follow button cycle; >=0 forced */
static float haptic_prev_angle = -1.0f;   /* for inst_vel differentiation */
static float haptic_inst_vel   =  0.0f;   /* fast filtered velocity [rad/s], used by detent mode */
static float smooth_vel        =  0.0f;   /* slow filtered velocity [rad/s], used by smooth mode */
static float smooth_v_filt = 0.0f;        /* IIR on voltage setpoint, smooth mode */
static float detent_v_filt = 0.0f;        /* IIR on voltage setpoint, detent mode */
static bool smooth_vdetent_armed = false;
static float smooth_vdetent_center = 0.0f;
static int64_t smooth_stationary_since_ms = 0;
static int smooth_nudge_dir = 0;
static int64_t smooth_nudge_start_ms = 0;
static int smooth_prev_step_index = 0;
static bool smooth_step_init = false;
static int detent_nudge_dir = 0;
static int64_t detent_nudge_start_ms = 0;
static bool detent_nudge_fired = false;

/* Diagnostic: counts every haptic_loop() iteration so the main thread can tell
 * whether the 1 kHz FOC thread is actually running (0 => thread never ran). */
static volatile uint32_t g_haptic_loop_count;

/* Diagnostic: counts how many times g_step_cb fired (knob navigation events). */
static volatile uint32_t g_step_fire_count;

uint32_t haptic_loop_count(void)
{
    return g_haptic_loop_count;
}

uint32_t haptic_step_fire_count(void)
{
    return g_step_fire_count;
}

float haptic_dbg_cumulative_angle(void)
{
    return cumulative_angle;
}

/* FOC control loop thread - triggered by k_timer at 10 kHz */
static bldc_motor_t *g_motor_ptr = NULL;
static bldc_driver_t *g_driver_ptr = NULL;
static sensor_t *g_encoder_ptr = NULL;
static K_SEM_DEFINE(haptic_sem, 0, 1);
static struct k_timer haptic_timer;

#define HAPTIC_THREAD_STACK_SIZE 4096
#define HAPTIC_THREAD_PRIORITY   0  /* Highest preemptible priority, safe at 1 kHz */
static K_THREAD_STACK_DEFINE(haptic_stack, HAPTIC_THREAD_STACK_SIZE);
static struct k_thread haptic_thread_data;

/* Diagnostic line buffer drained by disp_test.c at 100 ms cadence.
 * Keep this tiny and static to avoid heap use and log bursts on USB CDC. */
#define HAPTIC_DIAG_MAX_LINES 16
#define HAPTIC_DIAG_LINE_LEN  80
static char g_haptic_diag[HAPTIC_DIAG_MAX_LINES][HAPTIC_DIAG_LINE_LEN];
static int g_haptic_diag_count;

static void haptic_diag_reset(void)
{
    g_haptic_diag_count = 0;
}

static void haptic_diag_add(const char *line)
{
    if (line == NULL || g_haptic_diag_count >= HAPTIC_DIAG_MAX_LINES) {
        return;
    }

    (void)snprintf(g_haptic_diag[g_haptic_diag_count],
                   HAPTIC_DIAG_LINE_LEN,
                   "%s",
                   line);
    g_haptic_diag_count++;
}

int haptic_diag_count(void)
{
    return g_haptic_diag_count;
}

const char *haptic_diag_get_line(int idx)
{
    if (idx < 0 || idx >= g_haptic_diag_count) {
        return "";
    }
    return g_haptic_diag[idx];
}

int haptic_recover_after_ble(void)
{
    if (g_motor_ptr == NULL || g_driver_ptr == NULL || g_encoder_ptr == NULL) {
        return -ENODEV;
    }

    LOG_WRN("Recovering haptic HW after BLE startup transient");

    k_timer_stop(&haptic_timer);
    while (k_sem_take(&haptic_sem, K_NO_WAIT) == 0) {
    }

    bldc_motor_move(g_motor_ptr, 0.0f);
    bldc_motor_loop_foc(g_motor_ptr);

    int enc_ret = tmag5170_init((struct tmag5170_device *)g_encoder_ptr);
    int drv_ret = bldc_driver_3pwm_init_hw((bldc_driver_3pwm_t *)g_driver_ptr);

    sensor_update(g_encoder_ptr);
    start_angle = sensor_get_angle(g_encoder_ptr);
    haptic_prev_angle = -1.0f;
    haptic_inst_vel = 0.0f;
    smooth_vel = 0.0f;
    smooth_v_filt = 0.0f;
    detent_v_filt = 0.0f;
    cumulative_angle = 0.0f;
    smooth_vdetent_armed = false;
    smooth_vdetent_center = 0.0f;
    smooth_stationary_since_ms = 0;
    smooth_nudge_dir = 0;
    smooth_nudge_start_ms = 0;
    smooth_prev_step_index = 0;
    smooth_step_init = false;
    detent_prev_init = false;
    prev_detent_index = 0;
    detent_nudge_dir = 0;
    detent_nudge_start_ms = 0;
    detent_nudge_fired = false;

    k_timer_start(&haptic_timer, K_MSEC(1), K_MSEC(1));

    LOG_INF("BLE recovery done: TMAG=%d DRV=%d start=%.2f deg",
            enc_ret, drv_ret,
            (double)(start_angle * 180.0f / 3.14159f));

    if (enc_ret != 0) {
        return enc_ret;
    }
    if (drv_ret != DRIVER_INIT_OK) {
        return -EIO;
    }
    return 0;
}

/* Smoothly blend from 0 to 1 between lo and hi (Hermite interpolation).
 * Eliminates the hard on/off gate at the dead zone boundary that causes
 * the motor to buzz when encoder noise flips the gate condition every tick. */
static inline float haptic_blend(float x, float lo, float hi)
{
    if (x <= lo) return 0.0f;
    if (x >= hi) return 1.0f;
    float t = (x - lo) / (hi - lo);
    return t * t * (3.0f - 2.0f * t);  /* smoothstep / Hermite */
}

static void haptic_timer_cb(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&haptic_sem);
}

int haptic_get_num_steps(void)
{
    return g_num_steps_current;
}

void haptic_set_num_steps(int steps)
{
    if (steps < 0) {
        steps = 0;
    }
    g_num_steps_override = steps;
    g_num_steps_current = steps;
}

void haptic_set_step_callback(void (*cb)(int dir))
{
    g_step_cb = cb;
}

void haptic_set_virtual_click_callback(void (*cb)(int dir))
{
    g_virtual_click_cb = cb;
}

static void haptic_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    while (1) {
        k_sem_take(&haptic_sem, K_FOREVER);
        haptic_loop(g_motor_ptr);
    }
}

int haptic_update_num_steps_from_button(void)
{
    static bool initialized = false;
    static bool last_pressed = false;
    static int current_num_steps = 0;
    static int64_t last_change_ms = 0;

    if (!initialized) {
        if (user_button.port != NULL && gpio_is_ready_dt(&user_button)) {
            gpio_pin_configure_dt(&user_button, GPIO_INPUT);
            int init_state = gpio_pin_get_dt(&user_button);
            last_pressed = (init_state > 0);
            last_change_ms = k_uptime_get();
        }
        initialized = true;
    }

    if (user_button.port == NULL || !gpio_is_ready_dt(&user_button)) {
        return current_num_steps;
    }

    bool pressed = gpio_pin_get_dt(&user_button) > 0;
    int64_t now_ms = k_uptime_get();

    if (pressed && !last_pressed && (now_ms - last_change_ms) > 180) {
        /* Cycle: 0 -> 16 -> 12 -> 8 -> 0 -> 16 ... */
        if (current_num_steps == 0) {
            current_num_steps = 16;
        } else if (current_num_steps <= 8) {
            current_num_steps = 0;
        } else {
            current_num_steps -= 4;
        }
        last_change_ms = now_ms;
        LOG_INF("Steps: %d", current_num_steps);
    }

    last_pressed = pressed;
    return current_num_steps;
}

int haptic_init(bldc_motor_t *motor, bldc_driver_t *driver, sensor_t *encoder)
{
    bldc_driver_3pwm_t *driver_3pwm = (bldc_driver_3pwm_t *)driver;
    struct tmag5170_device *tmag = (struct tmag5170_device *)encoder;

    g_driver_ptr = driver;
    g_encoder_ptr = encoder;

    haptic_diag_reset();

    /* Initialize TMAG5170 encoder */
    LOG_INF("1. Initializing TMAG5170 encoder...");
    haptic_diag_add("diag: init encoder");
    if (tmag5170_init(tmag) < 0)
    {
        LOG_ERR("   Failed to initialize TMAG5170");
        haptic_diag_add("diag: encoder init failed");
        //return -1;
    }
    LOG_INF("   [OK] TMAG5170 ready");
    haptic_diag_add("diag: encoder ready");

    /* Initialize DRV8311H 3PWM Driver */
    LOG_INF("2. Initializing DRV8311H 3PWM driver...");
    haptic_diag_add("diag: init drv8311");
    bldc_driver_3pwm_init_struct(driver_3pwm);

    /* Configure driver parameters */
    driver_3pwm->pwm_frequency        = 25000;
    driver_3pwm->voltage_power_supply = SUPPLY_VOLTAGE;
    driver_3pwm->voltage_limit        = HAPTIC_VOLTAGE_LIMIT;

    if (bldc_driver_3pwm_init_hw(driver_3pwm) != DRIVER_INIT_OK)
    {
        LOG_ERR("   Failed to initialize driver");
        haptic_diag_add("diag: driver init failed");
        //return -1;
    }
    LOG_INF("   [OK] Driver initialized");
    haptic_diag_add("diag: driver ready");

    /* Initialize BLDC Motor */
    LOG_INF("3. Initializing BLDC motor...");
    bldc_motor_init_struct(motor,
                           MOTOR_POLE_PAIRS,
                           MOTOR_PHASE_RESISTANCE,
                           MOTOR_KV_RATING,
                           MOTOR_INDUCTANCE);

    /* Link driver to motor */
    bldc_motor_link_driver(motor, driver);

    /* Link sensor to motor */
    bldc_motor_link_sensor(motor, encoder);

    /* Configure motor parameters */
    motor->voltage_limit = HAPTIC_VOLTAGE_LIMIT;
    motor->velocity_limit = 20.0f;  /* rad/s */
    motor->voltage_sensor_align = 1.5f;

    if (!bldc_motor_init(motor))
    {
        LOG_ERR("   Failed to initialize motor");
        haptic_diag_add("diag: motor init failed");
        //return -1;
    }
    LOG_INF("   [OK] Motor initialized");
    haptic_diag_add("diag: motor ready");

    /* Run FOC calibration */
    LOG_INF("4. Running FOC calibration...");
    LOG_INF("   This will align sensor and motor phases");
    LOG_INF("   Motor will move slightly during calibration");
    k_msleep(1000);

    /* Inline zero-electric-angle measurement: ramp up voltage at 3pi/2,
     * hold, read encoder. Same call depth as the working settle loop below
     * (avoids stack overflow that hung inside bldc_motor_init_foc). */
    motor->sensor_direction = DIR_CW;
    motor->zero_electric_angle = 0.0f;  /* clear offset before measurement */
    LOG_INF("   Aligning rotor at 3pi/2...");
    for (int i = 0; i <= 50; i++) {
        float v = motor->voltage_sensor_align * (float)i / 50.0f;
        bldc_motor_set_phase_voltage(motor, v, 0.0f, 4.71239f); /* _3PI_2 */
        k_msleep(1);
    }
    k_msleep(700);  /* let rotor settle */
    sensor_update(motor->sensor);
    motor->zero_electric_angle = bldc_motor_electrical_angle(motor);
    bldc_motor_set_phase_voltage(motor, 0.0f, 0.0f, 0.0f);
    k_msleep(100);
    LOG_INF("   zero_electric_angle = %.4f rad", (double)motor->zero_electric_angle);

    /* bldc_motor_init_foc will now skip both loops (dir=CW, zero_el set) */
    if (!bldc_motor_init_foc(motor))
    {
        LOG_ERR("   FOC calibration failed");
        LOG_ERR("   Motor status: %d", motor->motor_status);
        haptic_diag_add("diag: foc failed");
        //return -1;
    }
    LOG_INF("   [OK] FOC calibration complete!");
    haptic_diag_add("diag: foc ok");

    LOG_INF("================================================");
    LOG_INF("  System Ready - Motor Status: %d", motor->motor_status);
    LOG_INF("  Entering main control loop...");
    LOG_INF("================================================");

    /* Set motor to torque control mode (voltage) */
    //motor->controller = TORQUE;
    motor->target = 0.0f;  /* Start with zero torque */

    /* Settle motor at zero torque after calibration */
    for (int i = 0; i < 100; i++) {
        bldc_motor_loop_foc(motor);
        bldc_motor_move(motor, 0.0f);
        k_msleep(1);
    }

    k_msleep(500);
    
    /* Store start angle for relative position calculation */
    sensor_update(encoder);
    start_angle = sensor_get_angle(encoder);
    LOG_INF("Encoder startup angle = %.2f deg",
            (double)(start_angle * 180.0f / 3.14159f));

    /* Encoder diagnostic: read 5 samples */
    LOG_INF("Encoder diagnostic (5 samples):");
    for (int i = 0; i < 5; i++) {
        k_msleep(100);
        sensor_update(encoder);
        float angle_rad = sensor_get_angle(encoder);
        LOG_INF("  [%d] angle=%.2f deg",
                i, (double)(angle_rad * 180.0f / 3.14159f));
    }

    num_steps_old = 0;

    LOG_INF("Starting motor control in 2 seconds...");
    haptic_diag_add("diag: haptic start");
    k_msleep(2000);

    /* Start 1 kHz FOC control loop. */
    g_motor_ptr = motor;
    /* DEBUG: timer/thread disabled, haptic_loop called from main */
    k_timer_init(&haptic_timer, haptic_timer_cb, NULL);
    k_thread_create(&haptic_thread_data, haptic_stack,
                   K_THREAD_STACK_SIZEOF(haptic_stack),
                   haptic_thread_fn, NULL, NULL, NULL,
                   HAPTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&haptic_thread_data, "haptic_foc");
    k_timer_start(&haptic_timer, K_MSEC(1), K_MSEC(1));  /* 1 kHz, matches reference */

    return 0;
}

void haptic_loop(bldc_motor_t *motor)
{
    g_haptic_loop_count++;
    int num_steps = haptic_update_num_steps_from_button();
    if (g_num_steps_override >= 0) {
        num_steps = g_num_steps_override;
    }
    g_num_steps_current = num_steps;

    sensor_update(motor->sensor);
    float current_angle = sensor_get_angle(motor->sensor);

    /* Instantaneous velocity: differentiate encoder angle, LPF alpha=0.1 at 1 kHz.
     * alpha=0.1 at 1 kHz gives about 16 Hz bandwidth (matches the reference project).
     * At 1 kHz a 1-LSB encoder spike is 0.38 rad/s vs 3.8 rad/s at 10 kHz. */
    if (haptic_prev_angle < 0.0f) haptic_prev_angle = current_angle;
    float d_ang = current_angle - haptic_prev_angle;
    while (d_ang >  _2PI * 0.5f) d_ang -= _2PI;
    while (d_ang < -_2PI * 0.5f) d_ang += _2PI;
    haptic_prev_angle = current_angle;
    /* Fast estimate for detent mode (16 Hz BW) */
    haptic_inst_vel = VEL_LPF_ALPHA * (d_ang * 1000.0f) + (1.0f - VEL_LPF_ALPHA) * haptic_inst_vel;
    /* Slow estimate for smooth mode (~1.6 Hz BW), stays non-zero between encoder ticks. */
    smooth_vel = SMOOTH_VEL_ALPHA * (d_ang * 1000.0f) + (1.0f - SMOOTH_VEL_ALPHA) * smooth_vel;
    cumulative_angle += d_ang;

    /* Log initial mode at very first call */
    static bool first_call = true;
    if (first_call) {
        first_call = false;
        LOG_INF("mode=%s num_steps=%d (initial)",
                (num_steps == 0) ? "smooth" : "detent", num_steps);
    }

    /* Preserve position when num_steps changes; reset IIR filters on transition */
    if (num_steps != num_steps_old) {
        num_steps_old = num_steps;
        smooth_v_filt = 0.0f;
        detent_v_filt = 0.0f;
        smooth_vel    = 0.0f;
        smooth_vdetent_armed = false;
        smooth_vdetent_center = cumulative_angle;
        smooth_stationary_since_ms = 0;
        smooth_nudge_dir = 0;
        smooth_nudge_start_ms = 0;
        smooth_step_init = false;
        detent_nudge_dir = 0;
        detent_nudge_start_ms = 0;
        detent_nudge_fired = false;
        detent_prev_init = false;
        LOG_INF("mode=%s num_steps=%d",
                (num_steps == 0) ? "smooth" : "detent", num_steps);
    }

    /* 3PWM: no phase-state management needed, just keep duty cycles active. */
    (void)motor->driver;  /* driver accessed through bldc_motor_set_phase_voltage */

    float target_voltage;

    if (num_steps == 0) {
        int64_t now_ms = k_uptime_get();
        float abs_vel = (smooth_vel < 0.0f) ? -smooth_vel : smooth_vel;
        float step_rad = SMOOTH_VDETENT_STEP_RAD;
        float nav_step_rad = SMOOTH_NAV_STEP_ANGLE_RAD;
        float rel_to_detent = cumulative_angle - smooth_vdetent_center;
        float abs_rel_to_detent = (rel_to_detent < 0.0f) ? -rel_to_detent : rel_to_detent;
        bool smooth_nav_locked = smooth_vdetent_armed && (abs_rel_to_detent < step_rad * 0.5f);

        /* ---- Navigation: always active, independent of arm state.
         * Counts mechanical steps exactly like detent mode; fires g_step_cb. */
        if (!smooth_nav_locked) {
            int step_index = (int)roundf(cumulative_angle / nav_step_rad);
            if (!smooth_step_init) {
                smooth_prev_step_index = step_index;
                smooth_step_init = true;
            }
            int delta = step_index - smooth_prev_step_index;
            if (delta != 0) {
                int dir = (delta > 0) ? 1 : -1;
                int steps = (delta > 0) ? delta : -delta;
                for (int i = 0; i < steps; i++) {
                    g_step_fire_count++;
                    if (g_step_cb) {
                        g_step_cb(dir);
                    }
                }
                smooth_prev_step_index = step_index;
            }
        } else {
            /* Freeze incremental navigation while user is still in the virtual detent.
             * This prevents accidental pre-scroll before the detent is actually exited. */
            smooth_prev_step_index = (int)roundf(cumulative_angle / nav_step_rad);
            smooth_step_init = true;
        }

        /* ---- Arm virtual detent after knob comes to rest. ----
         * Disarm only when turning fast so detent survives slow sweeps. */
        if (abs_vel <= SMOOTH_VDETENT_ARM_VEL) {
            if (smooth_stationary_since_ms == 0) {
                smooth_stationary_since_ms = now_ms;
            }
            if (!smooth_vdetent_armed && (now_ms - smooth_stationary_since_ms) >= SMOOTH_VDETENT_ARM_MS) {
                smooth_vdetent_armed = true;
                smooth_vdetent_center = cumulative_angle;
                smooth_nudge_dir = 0;
                smooth_nudge_start_ms = now_ms; /* start hold timer immediately */
            }
        } else {
            smooth_stationary_since_ms = 0;
            if (abs_vel > SMOOTH_VDETENT_ARM_VEL * 8.0f) {
                /* Disarm only on clearly fast rotation */
                smooth_vdetent_armed = false;
                smooth_nudge_dir = 0;
                smooth_nudge_start_ms = 0;
            }
        }

        /* ---- Smooth mode base force: viscous damping ---- */
        float v_sp = 0.0f;
        if (abs_vel > SMOOTH_VEL_THRESH) {
            float scale = (abs_vel - SMOOTH_VEL_THRESH) / SMOOTH_VEL_THRESH;
            if (scale > 1.0f) scale = 1.0f;
            v_sp = -SMOOTH_KV * smooth_vel * scale;
        }

        /* ---- Virtual detent spring + click (only when armed) ---- */
        if (smooth_vdetent_armed) {
            float rel = cumulative_angle - smooth_vdetent_center;
            float abs_rel = (rel < 0.0f) ? -rel : rel;
            float norm_err = rel / step_rad;

            if (abs_rel < step_rad * 0.5f) {
                /* Inside spring zone: apply restoring force with a smooth blend-in region.
                 * Below HAPTIC_ZERO_ZONE_RAD: force is zero (no PWM jitter at true centre).
                 * Between HAPTIC_ZERO_ZONE_RAD and HAPTIC_BLEND_ZONE_RAD: force ramps in via
                 * smoothstep so there is no abrupt voltage step that causes motor buzz. */
                {
                    float blend = haptic_blend(abs_rel, HAPTIC_ZERO_ZONE_RAD, HAPTIC_BLEND_ZONE_RAD);
                    v_sp += -blend * HAPTIC_DETENT_AMP_V * sinf(_2PI * norm_err);
                }

                /* Virtual click: only evaluate in outer gesture zone. */
                {
                    float abs_norm = (norm_err < 0.0f) ? -norm_err : norm_err;
                    int push_dir = (rel < 0.0f) ? -1 : 1;

                    if (abs_norm >= GESTURE_ARM_ZONE_MIN) {
                        if (smooth_nudge_dir == 0 || smooth_nudge_dir != push_dir) {
                            smooth_nudge_dir = push_dir;
                            smooth_nudge_start_ms = now_ms;
                        } else if ((now_ms - smooth_nudge_start_ms) >= VCLICK_HOLD_MS) {
                            if (g_virtual_click_cb) {
                                g_virtual_click_cb(smooth_nudge_dir);
                            }
                            smooth_nudge_dir = 0;
                            smooth_nudge_start_ms = 0;
                            smooth_vdetent_center = cumulative_angle;
                        }
                    } else if (abs_norm <= GESTURE_RELEASE_ZONE) {
                        smooth_nudge_dir = 0;
                        smooth_nudge_start_ms = 0;
                    }
                }
            } else {
                /* Past spring boundary: consume as navigation step and force re-arm at rest.
                 * This prevents virtual click from triggering after the snap. */
                smooth_nudge_dir = 0;
                smooth_nudge_start_ms = 0;
                smooth_vdetent_armed = false;
                smooth_stationary_since_ms = 0;
            }
        }

        if (v_sp >  motor->voltage_limit) v_sp =  motor->voltage_limit;
        if (v_sp < -motor->voltage_limit) v_sp = -motor->voltage_limit;

        smooth_v_filt = SMOOTH_IIR_ALPHA * v_sp + (1.0f - SMOOTH_IIR_ALPHA) * smooth_v_filt;
        target_voltage = smooth_v_filt;
    } else {
        /* ---- Detent mode: sinusoidal spring + velocity damping ---- */
        float step_size  = _2PI / (float)num_steps;
        float normalized = cumulative_angle / step_size;
        float nearest_f  = roundf(normalized);
        int current_detent_index = (int)nearest_f;

        /* Incremental encoder-style output: emit +1/-1 per crossed detent. */
        if (!detent_prev_init) {
            prev_detent_index = current_detent_index;
            detent_prev_init = true;
        }
        int delta = current_detent_index - prev_detent_index;
        if (delta != 0) {
            int dir = (delta > 0) ? 1 : -1;
            int steps = (delta > 0) ? delta : -delta;
            for (int i = 0; i < steps; i++) {
                g_step_fire_count++;
                if (g_step_cb) {
                    g_step_cb(dir);
                }
            }

            prev_detent_index = current_detent_index;
        }

        /* Error relative to nearest detent center in range (-0.5, 0.5).
         * The spring term -A*sin(2*pi*err) restores toward zero. */
        float norm_err = normalized - nearest_f;

        /* Virtual click dead zone: arm only when knob is held near the snap point
         * (|norm_err| >= GESTURE_ARM_ZONE_MIN).
         * Must exit the zone (return toward centre) before re-arming after a trigger. */
        {
            float abs_norm_err = (norm_err < 0.0f) ? -norm_err : norm_err;
            int snap_dir = (norm_err > 0.0f) ? 1 : -1;

            if (abs_norm_err >= GESTURE_ARM_ZONE_MIN) {
                if (detent_nudge_fired) {
                    /* Already triggered once. Wait for user to return to center. */
                } else if (detent_nudge_dir == 0 || snap_dir != detent_nudge_dir) {
                    detent_nudge_dir = snap_dir;
                    detent_nudge_start_ms = k_uptime_get();
                } else if ((k_uptime_get() - detent_nudge_start_ms) >= VCLICK_HOLD_MS) {
                    if (g_virtual_click_cb) {
                        g_virtual_click_cb(detent_nudge_dir);
                    }
                    detent_nudge_dir = 0;
                    detent_nudge_start_ms = 0;
                    detent_nudge_fired = true;
                    /* Reserved: check (now - start) >= VCLICK_LONG_HOLD_MS for TODO 2.b */
                }
            } else {
                detent_nudge_dir = 0;
                detent_nudge_start_ms = 0;
                detent_nudge_fired = false;
            }
        }

        float v_sp = -DETENT_KV * haptic_inst_vel;
        /* Smooth blend-in to avoid oscillation at the dead zone edge. */
        {
            float abs_err_rad = fabsf(norm_err * step_size);
            float blend = haptic_blend(abs_err_rad, HAPTIC_ZERO_ZONE_RAD, HAPTIC_BLEND_ZONE_RAD);
            v_sp += -blend * HAPTIC_DETENT_AMP_V * sinf(_2PI * norm_err);
        }
        if (v_sp >  motor->voltage_limit) v_sp =  motor->voltage_limit;
        if (v_sp < -motor->voltage_limit) v_sp = -motor->voltage_limit;

        detent_v_filt = DETENT_IIR_ALPHA * v_sp + (1.0f - DETENT_IIR_ALPHA) * detent_v_filt;
        target_voltage = detent_v_filt;

        /* Periodic diagnostic: confirm encoder tracking + spring (every 200 ms) */
        static int det_log_cnt = 0;
        if (++det_log_cnt >= 200) {
            det_log_cnt = 0;
            LOG_INF("det: ang=%.3f cum=%.3f ne=%.3f vsp=%.3f tv=%.3f",
                    (double)sensor_get_angle(motor->sensor),
                    (double)cumulative_angle,
                    (double)norm_err, (double)v_sp, (double)target_voltage);
        }
    }

    /* move() sets target, loop_foc() applies it using fresh electrical angle.
     * Order MUST be: move -> loop_foc (SimpleFOC contract). */
    bldc_motor_move(motor, target_voltage);
    bldc_motor_loop_foc(motor);
}
