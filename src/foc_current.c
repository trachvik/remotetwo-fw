/*
 * foc_current.c
 * Standalone application for testing closed-loop current FOC control.
 */

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <hal/nrf_gpio.h>

/* Adjust these includes based on your project structure */
#include "drivers/tmag5170_sensor.h"
#include "drivers/current_sense.h"
#include "drivers/bldc_motor.h"
#include "drivers/bldc_driver_3pwm.h"
#include "drivers/anticogging.h"

LOG_MODULE_REGISTER(foc_current_test, LOG_LEVEL_INF);

/* Replicate the proven power-rail sequencing from ui_display.c:
 * - Enable CTRL (P0.30) early so SPI peripherals on this rail are powered.
 * - Keep MIC2288 boost (P0.28) OFF for standalone FOC test.
 * - Keep OLED held quiet on the shared SPI bus.
 * Run before TMAG sensor init (SENSOR init priority is 90). */
static int foc_power_rails_and_mute_oled(void)
{
    const uint32_t oled_cs  = NRF_GPIO_PIN_MAP(0, 2);   /* CSn_OLED  (active low) */
    const uint32_t oled_rst = NRF_GPIO_PIN_MAP(1, 14);  /* OLED_RST  (active low) */
    const uint32_t disp_mos = NRF_GPIO_PIN_MAP(0, 28);  /* MIC2288_MOS (active high) */
    const uint32_t disp_pwr = NRF_GPIO_PIN_MAP(0, 30);  /* CTRL        (active high) */

    /* Keep panel boost OFF in this test app. */
    nrf_gpio_cfg_output(disp_mos);
    nrf_gpio_pin_clear(disp_mos);

    /* Enable logic power rail used by board peripherals (critical for TMAG). */
    nrf_gpio_cfg_output(disp_pwr);
    nrf_gpio_pin_set(disp_pwr);

    k_msleep(20);

    nrf_gpio_cfg_output(oled_cs);
    nrf_gpio_pin_set(oled_cs);   /* CS high -> OLED tri-state on SPI */

    nrf_gpio_cfg_output(oled_rst);
    nrf_gpio_pin_clear(oled_rst); /* Hold display in reset */

    return 0;
}
SYS_INIT(foc_power_rails_and_mute_oled, POST_KERNEL, 10);

/* 1 = run the physical alignment sweep and PERSIST the result to flash.
 * 0 = skip it and load the stored zero-electric-angle back from flash.
 * Set to 1 for one boot after any change that alters the applied field (phase
 * swap, modulation change, magnet/sensor reseat), then back to 0. */
#define FOC_DO_CALIBRATION 1
#define ANTICOGGING_COMPENSATION 1 // whether to use the anticogging map for feedforward compensation in velocity control loop

#define SUPPLY_VOLTAGE 3.75f
#define HAPTIC_VOLTAGE_LIMIT SUPPLY_VOLTAGE

/* FOC loop rate [Hz]; period = 200 µs */
#define HAPTIC_LOOP_HZ 5000U 

/* Motor parameters */
#define MOTOR_POLE_PAIRS       11
/* PER-PHASE series resistance seen by the current loop [Ω].
 * The GM3506 datasheet's "internal resistance 5.6 Ω" is the LINE-TO-LINE value;
 * per phase that is 2.8 Ω. Add ~0.3 Ω for the DRV8311H RDS(on) + shunt in the
 * loop => ~3.1 Ω. Confirmed by bldc_motor_align_current_sense(): 1.81 V lands
 * across phase A and 0.582 A was measured => 3.11 Ω.
 * This value directly sets Ki = R*wc, whose zero must cancel the electrical
 * pole at R/L. Using 5.6 put the zero at 2500 rad/s against a pole at
 * 1384 rad/s - no cancellation, crossover pushed to ~110 Hz, and with the
 * measurement delay the loop sat at ~0 deg phase margin (the standstill buzz). */
#define MOTOR_PHASE_RESISTANCE 3.1f
#define MOTOR_KV_RATING        141.0f  /* rpm/V */
/* H. NOTE: 2.24 mH came from L_MEAS below, which divided by the (wrong) 5.6 Ω
 * and resolved tau with only 200 µs steps - so it is good to a factor of ~2 at
 * best. Scaled by the corrected resistance it is ~1.24 mH. Kp = L*wc only sets
 * the crossover, so an error here shifts the bandwidth but does not destabilise
 * the loop; re-measure properly if the step response is not ~2 ms. */
#define MOTOR_INDUCTANCE       0.00124f

/* Current loop tuning.
 * IMC: Kp = L·2π·BW, Ki = R·2π·BW. With R correct the PI zero cancels the
 * motor pole exactly and the open loop becomes a pure integrator crossing at
 * BW, so the only remaining phase eaters are the measurement LPF and the
 * SAADC transport delay. With SCANS_PER_BUF=5 (200 µs refresh) the delay is
 * ~300 µs = 8.6 deg at 80 Hz, and a 400 Hz LPF costs 11 deg => ~70 deg margin.
 * The old 160 Hz LPF (Tf = 995 µs) alone cost 35 deg. */
#define HAPTIC_CURRENT_LOOP_BW_HZ    80.0f
#define HAPTIC_CURRENT_LPF_CUTOFF_HZ 400.0f
#define HAPTIC_CURRENT_LIMIT         0.5f  /* A, hard clamp on q-current setpoint for safe bring-up */

/* One-shot inductance measurement (step-voltage method); see measure_inductance() below */
#define L_MEAS_ENABLE  0   /* measurement done: L=2.24 mH */
#define L_MEAS_V_STEP  0.5f   /* V  - small enough not to move the rotor */
#define L_MEAS_MS      5      /* ms - 5× expected τ is enough (τ < 1 ms for most small motors) */
static void measure_inductance(void);

/* ---- LOCKED-ROTOR current-loop characterisation ----------------------------
 * Everything below is driven with bldc_motor_set_phase_voltage() at a FIXED
 * electrical angle of 0, and the phase currents are Park-transformed at that
 * SAME fixed angle. That takes the encoder, zero_electric_angle and the rotor
 * dynamics out of the measurement completely: a d-axis voltage at angle 0
 * pulls the rotor to angle 0 and holds it there, and once parked a d-axis
 * current produces no torque. What remains is a pure R-L circuit.
 *
 * At angle 0 the phase-to-neutral voltages are (ud, -ud/2, -ud/2), so the
 * measured i_d is exactly i_a = ud / R_phase. R and L therefore fall straight
 * out of the step response with no calibration constants in the way.
 * Conveniently, phase A - the high-duty one at this angle - is the
 * reconstructed phase, so both measured shunts sit near 32% duty, safely
 * inside the low-side sampling window.
 *
 * Pass A measures R and L open-loop, reconfigures the PI from them, then
 * pass B runs a closed-loop d-current step with those gains.
 *
 * The 5 kHz capture stays in RAM and only the extracted figures are printed -
 * dumping 1000 samples per pass scrolled the boot log out of the terminal. */
#define FOC_STEP_TEST   1
#define STEP_V_LOW      0.30f  /* V, d-axis pre-lock and low level of the step */
#define STEP_V_HIGH     0.90f  /* V, high level; keeps every duty below ~0.70  */
#define STEP_I_LOW      0.10f  /* A, closed-loop low level (rotor stays locked) */
#define STEP_I_HIGH     0.25f  /* A, closed-loop high level                     */
#define STEP_HALF_MS    25     /* half-period of the square wave [ms]           */
#define STEP_SAMPLES    1000   /* 200 ms at 5 kHz = 8 half-periods              */
#define STEP_HALF_TICKS (STEP_HALF_MS * (HAPTIC_LOOP_HZ / 1000U))
#if FOC_STEP_TEST
static void run_current_step_test(void);
#endif

/* Global hardware instances for this standalone test */
static struct tmag5170_device encoder;
static bldc_driver_3pwm_t driver;
static bldc_motor_t motor;

/* FOC control loop thread data */
static K_SEM_DEFINE(haptic_sem, 0, 1);
static struct k_timer haptic_timer;

#define HAPTIC_THREAD_STACK_SIZE 4096
#define HAPTIC_THREAD_PRIORITY   0 
static K_THREAD_STACK_DEFINE(haptic_stack, HAPTIC_THREAD_STACK_SIZE);
static struct k_thread haptic_thread_data;

/**
 * @brief Timer callback - gives the semaphore to trigger the FOC loop
 */
static void haptic_timer_cb(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&haptic_sem);
}

/**
 * @brief The actual FOC loop executed 5000 times per second
 */
static void haptic_loop(bldc_motor_t *m)
{
    /* 1. Update sensor */
    sensor_update(m->sensor);

    // 2. VELOCITY LOOP - DOWNSAMPLING to 100 Hz 
    /*static int move_divider = 0;
    if (++move_divider >= 50) { 
        move_divider = 0;
        float target_velocity_rad_s = 0.0f; 
        bldc_motor_move(m, target_velocity_rad_s);
    }*/

    // TORQUE mode:
    // debug:
    //static int debug_devider = 0;
    //static float ff_factor = -2.0f;

    bldc_motor_move(m, 0.0f);
    
    /* Save the pure PID value for diagnostics/logging */
    float original_sp = m->current_sp; 

    #if ANTICOGGING_COMPENSATION
        /* 3. ANTICOGGING INJECTION WITH FADE-OUT */
        float raw_ff = anticogging_get_feedforward(sensor_get_angle(m->sensor));
        
        /* Absolute value of the current rotor velocity */
        //float abs_vel = (m->shaft_velocity < 0.0f) ? -m->shaft_velocity : m->shaft_velocity;
        //float fade_factor = 1.0f;
        
        /* Start fading out at 1.5 rad/s, disable completely at 3.0 rad/s */
        /*float fade_start = 1.5f; 
        float fade_end = 3.0f;
        
        if (abs_vel >= fade_end) {
            fade_factor = 0.0f;
        } else if (abs_vel > fade_start) {
            // Linear fade-out from 1.0 to 0.0 
            fade_factor = 1.0f - ((abs_vel - fade_start) / (fade_end - fade_start));
        }*/
        /*if (debug_devider++ >= 5000*5) { // every 5 seconds
            debug_devider = 0;
            ff_factor += 0.2f;
            if (ff_factor > 2.0f) ff_factor = -2.0f;
            printk("DEBUG: ff_factor = %f\n", (double)ff_factor);
        }*/
        
        //m->current_sp += (raw_ff * fade_factor); 
        m->current_sp += 0.8f * raw_ff;
    #endif

    /* 4. CURRENT LOOP */
    bldc_motor_loop_foc(m);

    /* Restore the pure PID value so it doesn't get written back into the structure */
    m->current_sp = original_sp; 

    // --- SERIALPLOT LOGGING (100 Hz) --- 
   /* static int log_counter = 0;
    if (++log_counter >= 50) { 
        log_counter = 0;
        float raw_vel = sensor_get_velocity(m->sensor);
        float bx_mt = sensor_get_bx_mt(m->sensor);
        float by_mt = sensor_get_by_mt(m->sensor);
    
        printk("DATA:%f,%f,%f,%f,%f\n", 
            (double)raw_vel,             
            (double)m->shaft_velocity,   
            (double)(original_sp * 10.0f),
            (double)bx_mt,
            (double)by_mt); */
 
               /* Plotting original_sp! In SerialPlot, you should see the 
                * green wave gradually shrinking to zero, as the mapping (ff) 
                * takes over overcoming the cogging instead of the controller. */
    //}
}

/**
 * @brief FOC thread function - waits for the timer semaphore and runs the FOC loop
 */
static void haptic_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    while (1) {
        k_sem_take(&haptic_sem, K_FOREVER);
        haptic_loop(&motor);
    }
}

/**
 * @brief Initialize all hardware and start the FOC thread
 */
static int standalone_haptic_init(void)
{
    /* 1. Initialize TMAG5170 encoder */
    LOG_INF("1. Initializing TMAG5170 encoder...");
    int enc_err = tmag5170_init(&encoder);
    if (enc_err < 0) {
        LOG_ERR("  [FAIL] TMAG5170 init failed: %d", enc_err);
        return enc_err;
    }

    /* 2. Initialize DRV8311H 3PWM Driver */
    LOG_INF("2. Initializing DRV8311H 3PWM driver...");
    bldc_driver_3pwm_init_struct(&driver);
    driver.pwm_frequency        = 25000;
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit        = HAPTIC_VOLTAGE_LIMIT;
    bldc_driver_3pwm_init_hw(&driver);

    /* 3. Initialize BLDC Motor */
    LOG_INF("3. Initializing BLDC motor...");
    bldc_motor_init_struct(&motor, MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV_RATING, MOTOR_INDUCTANCE);
    bldc_motor_link_driver(&motor, (bldc_driver_t *)&driver);
    bldc_motor_link_sensor(&motor, (sensor_t *)&encoder);

    /* Set voltage limits (0.58 margin is required for SVPWM without saturation) */
    motor.voltage_limit = HAPTIC_VOLTAGE_LIMIT*0.58f; // 0.58 margin is required for SVPWM without saturation
    motor.velocity_limit = 20.0f;
    motor.voltage_sensor_align = SUPPLY_VOLTAGE; /* ALWAYS calibrate with max voltage to overcome cogging */
    bldc_motor_init(&motor);

    /* 4. Run FOC calibration or set fixed values */

#if FOC_DO_CALIBRATION
    /* SimpleFOC alignSensor sequence, internal. The direction-detection sweep is run
     * (sensor_direction left UNKNOWN) because that physical movement exercises the
     * bearing and makes the following zero-angle measurement more repeatable. The
     * measured zero-electric-angle is then PERSISTED to flash so a non-calibration
     * boot can read it back and skip the slow, run-to-run-varying alignment.
     * Runtime direction is pinned to DIR_CW afterwards. */
    LOG_INF("4. Running FOC calibration...");
    k_msleep(1000);
    motor.sensor_direction = DIR_UNKNOWN;   /* run dir-detect sweep (movement) */
    bldc_motor_init_foc(&motor);
    motor.sensor_direction = DIR_CW;        /* pin runtime direction */
    if (motor.motor_status != MOTOR_READY) {
        LOG_ERR("  [FAIL] FOC calibration failed (status %d) - zero angle NOT stored.",
                (int)motor.motor_status);
        return -EIO;
    }
    if (anticogging_flash_save_zero_angle(motor.zero_electric_angle) == 0) {
        LOG_INF("  [OK] Zero angle %.3f STORED to flash.", (double)motor.zero_electric_angle);
    } else {
        LOG_ERR("  [FAIL] Could not store zero angle to flash!");
    }
    LOG_INF("  [OK] FOC calibration complete!");
#else
    /* Calibration disabled -> skip the physical alignment and read the previously
     * measured zero electric angle from flash. Presetting zero_electric_angle makes
     * init_foc skip its own alignment sweep. */
    LOG_INF("4. Skipping calibration, loading zero angle from flash...");
    motor.sensor_direction = DIR_CW;   /* must match the direction used at calibration */
    float stored_zero_angle = 0.0f;
    int zero_angle_rc = anticogging_flash_load_zero_angle(&stored_zero_angle);
    if (zero_angle_rc != 0) {
        LOG_ERR("  [FAIL] No zero angle in flash - run FOC_DO_CALIBRATION once!");
        return zero_angle_rc;
    }
    motor.zero_electric_angle = stored_zero_angle;
    LOG_INF("  [OK] Zero angle %.3f loaded from flash.", (double)stored_zero_angle);
    bldc_motor_init_foc(&motor);   /* alignment skipped (zero angle already set) */
    LOG_INF("  [OK] FOC initialized!");
#endif

    /* 5. Initialize FOC current sense (DRV8311H CSA via SAADC) */
    LOG_INF("5. Initializing FOC current sense...");
    int cs_err = current_sense_init();

    /* =========================================================================
     * HARDWARE WORKAROUND: ADC WARMUP
     * Pulse PWM in safe VOLTAGE mode (0.0V) to stabilize the SAADC and DPPI 
     * chains BEFORE measuring the zero-current offsets. Prevents phantom currents.
     * ========================================================================= */
    motor.torque_controller = VOLTAGE; 
    motor.target = 0.0f;
    for (int i = 0; i < 100; i++) {
        bldc_motor_loop_foc(&motor);
        bldc_motor_move(&motor, 0.0f);
        k_msleep(1);
    }
    k_msleep(200);
    sensor_update(motor.sensor);
    /* ========================================================================= */
    
    if (cs_err == 0 && current_sense_is_ready()) {
        current_sense_calibrate_offsets();
        
        /* Configure Current PID */
        motor.pid_current_q.limit = motor.voltage_limit;
        motor.pid_current_d.limit = motor.voltage_limit;
        motor.current_limit = HAPTIC_CURRENT_LIMIT;

        bldc_motor_configure_current_loop(&motor, 
                                          HAPTIC_CURRENT_LOOP_BW_HZ, 
                                          (float)HAPTIC_LOOP_HZ, 
                                          HAPTIC_CURRENT_LPF_CUTOFF_HZ);

        // ------------- NEW: VELOCITY LOOP -------------
        /* DIAGNOSTICS NOTE: 
         * Controller parameters (P=0.01) and filter (tf=0.015) were adjusted 
         * for anticogging needs. The uncompensated run graph therefore has a 
         * different shape (sharper and softer) than in older versions, which is 
         * desirable for better visibility of real hardware anomalies. */
        motor.pid_velocity.p = 0.01f;     // Slightly stronger response
        motor.pid_velocity.i = 0.2f;      
        motor.pid_velocity.limit = 0.25f; // Allow more effort to overcome the cogging tooth
        
        /* Low-pass filter to smooth out heavy sensor noise */
        motor.lpf_velocity.tf = 0.015f;    // Stronger noise filtering
        motor.lpf_velocity.Ts = 0.01f;    // FIX 1: Hardcode the time step (100 Hz = 10 ms)
        
        /* Control mode change: Outer is velocity, inner is current */
        motor.controller = TORQUE; 
        motor.torque_controller = FOC_CURRENT;

        /* FIX 2: Hardcode the direction so the velocity multiplier isn't zero! */
        //motor.sensor_direction = DIR_CW;  // (If it spins the wrong way, change to DIR_CCW)
        // ---------------------------------------------------

        //#if FOC_DO_CALIBRATION
            bldc_motor_align_current_sense(&motor);
        //#else
         //   current_sense_set_gain_sign(-1.0f);
        //#endif
        /* Keep current mode active: anticogging calibration commands amps. */
        motor.torque_controller = FOC_CURRENT;
        printk("  [OK] FOC_CURRENT active!");

#if L_MEAS_ENABLE
        measure_inductance();
#endif
    } else {
        LOG_ERR("  [FAIL] SAADC not ready, cannot test current control!");
        return -1; /* Stop execution if ADC is dead, no point in continuing */
    }

    //printk("---> [DEBUG 2] Starting 5kHz timer...\n");
    k_timer_init(&haptic_timer, haptic_timer_cb, NULL);
    k_timer_start(&haptic_timer, K_USEC(1000000U / HAPTIC_LOOP_HZ), K_USEC(1000000U / HAPTIC_LOOP_HZ));

    //printk("---> [DEBUG 3] Calling calibration...\n");
    k_msleep(10); /* Give the processor a 10 ms pause to catch its breath */

#if FOC_STEP_TEST
    /* Verify the inner current loop BEFORE spending 20 minutes calibrating on
     * top of it: an unstable current loop makes every anticogging measurement
     * meaningless. */
    run_current_step_test();
#endif

    anticogging_calibrate(&motor, &haptic_sem);

    /* Step-test / rest-probe coast Hi-Z. Haptic must have the bridge on. */
    bldc_motor_reset_current_loop_state(&motor);
    bldc_motor_set_bridge_hiz(&motor, false);

    //printk("---> [DEBUG 4] Calibration done, creating thread...\n");
    
    k_thread_create(&haptic_thread_data, haptic_stack, K_THREAD_STACK_SIZEOF(haptic_stack),
                    haptic_thread_fn, NULL, NULL, NULL, HAPTIC_THREAD_PRIORITY, K_FP_REGS, K_NO_WAIT);
    k_thread_name_set(&haptic_thread_data, "haptic_foc");

    return 0;
}

/* =========================================================================
 * ONE-SHOT INDUCTANCE MEASUREMENT (step-voltage method)
 * Motor must be at standstill, electrical angle locked to 0 before calling.
 * Applies V_step on the alpha axis, samples current for MEAS_MS, fits τ = L/R.
 * ========================================================================= */

static void measure_inductance(void)
{
#if L_MEAS_ENABLE
    const int samples = L_MEAS_MS * 1000 / 200; /* at 5 kHz = 200 µs per step */
    const float Ts = 0.0002f;
    const float R  = MOTOR_PHASE_RESISTANCE;
    const float V  = L_MEAS_V_STEP;
    const float I_ss = V / R; /* expected steady-state current */

    /* Lock rotor at electrical angle 0 with a tiny d-axis voltage. */
    bldc_motor_set_phase_voltage(&motor, 0.0f, V, 0.0f);
    k_msleep(L_MEAS_MS * 5);  /* let rotor settle before we start */

    /* Step: switch on alpha voltage and record when current crosses 63 % of I_ss. */
    bldc_motor_set_phase_voltage(&motor, 0.0f, 0.0f, 0.0f);
    k_msleep(2); /* discharge */

    float tau_est = 0.0f;
    bool  found   = false;
    bldc_motor_set_phase_voltage(&motor, 0.0f, V, 0.0f);

    for (int i = 0; i < samples * 10 && !found; i++) {
        k_usleep(200);
        phase_current_t c = current_sense_get_phase_currents();
        float ia = c.a * current_sense_get_gain_sign();
        if (ia >= 0.632f * I_ss) {
            tau_est = (float)(i + 1) * Ts; /* i+1: sleep runs before read */
            found   = true;
        }
    }
    bldc_motor_set_phase_voltage(&motor, 0.0f, 0.0f, 0.0f);

    if (!found) {
        printk("L_MEAS: current never reached 63%% of I_ss=%.3f A "
               "- increase L_MEAS_V_STEP or L_MEAS_MS\n", (double)I_ss);
        return;
    }

    float L_meas = tau_est * R;
    printk("L_MEAS: tau=%.3f ms  L=%.1f uH  R=%.2f Ohm\n",
           (double)(tau_est * 1e3f), (double)(L_meas * 1e6f), (double)R);

    /* Suggest gains for common bandwidths. */
    for (int bw = 5; bw <= 50; bw += 5) {
        float wc = 2.0f * 3.14159f * (float)bw;
        printk("  BW=%2d Hz -> Kp=%.5f  Ki=%.1f\n",
               bw, (double)(L_meas * wc), (double)(R * wc));
    }
#endif
}

#if FOC_STEP_TEST

static int16_t step_log[STEP_SAMPLES][4];

/* Headroom, in 16 MHz PWM ticks, left in the low-side conduction window after
 * the SAADC has finished sampling the two measured shunts. NEGATIVE means at
 * least one shunt was sampled while its high-side FET was conducting, so that
 * reading - and every current derived from it - is invalid.
 *
 * The period is 640 ticks (25 kHz). With PWM_LOWSIDE_SYNC the low-side window
 * is [0, (1-duty)*640). The scan starts at tick 25 and each channel costs 3 us
 * acquisition + ~2 us conversion = 80 ticks, so SOC (ch0) completes at tick 105
 * and SOB (ch1) at tick 185. */
static int step_sense_slack(void)
{
    float slack_c = (1.0f - driver.dc_c) * 640.0f - 105.0f;
    float slack_b = (1.0f - driver.dc_b) * 640.0f - 185.0f;
    float s = (slack_b < slack_c) ? slack_b : slack_c;
    if (s >  1000.0f) s =  1000.0f;
    if (s < -1000.0f) s = -1000.0f;
    return (int)s;
}

/* Smallest slack seen anywhere in the capture (channel 4 of every record). */
static int step_min_slack(void)
{
    int m = 32767;
    for (int n = 0; n < STEP_SAMPLES; n++) {
        if (step_log[n][3] < m) m = step_log[n][3];
    }
    return m;
}

/* Ensemble-average the response to every rising edge in the capture into
 * avg[0..STEP_HALF_TICKS), and return the mean level just before those edges.
 * Averaging the 7-8 transitions cuts the ADC noise ~3x, which matters because
 * the time constant we are after is only ~4 samples long.
 * Column `col` is the signal, column 0 is the level used for edge detection. */
static float step_ensemble(float *avg, int16_t edge_thresh, int col, int *n_edges)
{
    for (int k = 0; k < (int)STEP_HALF_TICKS; k++) avg[k] = 0.0f;

    float lo_sum = 0.0f;
    int   edges  = 0;

    for (int n = 20; n + (int)STEP_HALF_TICKS <= STEP_SAMPLES; n++) {
        if (step_log[n][0] < edge_thresh || step_log[n - 1][0] >= edge_thresh) continue;

        for (int k = 0; k < (int)STEP_HALF_TICKS; k++) {
            avg[k] += (float)step_log[n + k][col];
        }
        for (int k = 1; k <= 20; k++) lo_sum += (float)step_log[n - k][col];
        edges++;
    }

    if (edges > 0) {
        for (int k = 0; k < (int)STEP_HALF_TICKS; k++) avg[k] /= (float)edges;
        lo_sum /= (float)(edges * 20);
    }
    *n_edges = edges;
    return lo_sum;
}

/* Locked-rotor characterisation. Must be called AFTER the haptic timer is
 * running - both passes pace off haptic_sem at the 5 kHz FOC rate. */
static void run_current_step_test(void)
{
    static float avg[STEP_HALF_TICKS];

    if (!current_sense_is_ready()) {
        printk("STEP TEST skipped: current sense not ready\n");
        return;
    }

    printk("\n=== LOCKED-ROTOR CURRENT-LOOP TEST ===\n");

    /* Park the rotor at electrical angle 0: ramp the d-axis voltage up slowly
     * so it creeps onto the detent instead of snapping, then let it stop
     * ringing. From here on d-axis current makes no torque. */
    for (int i = 0; i <= 200; i++) {
        bldc_motor_set_phase_voltage(&motor, 0.0f, STEP_V_LOW * (float)i / 200.0f, 0.0f);
        k_msleep(2);
    }
    k_msleep(400);

    /* ---------------- Pass A: open-loop R-L step ------------------------- */
    {
        uint32_t cnt  = 0;
        bool     high = false;
        float    ud   = STEP_V_LOW;   /* voltage currently on the bridge */

        for (int n = 0; n < STEP_SAMPLES; n++) {
            k_sem_take(&haptic_sem, K_FOREVER);

            /* Read first: this sample is the consequence of the voltage that
             * was already on the bridge, so the logged (u, i) pair is causal
             * and the extracted time constant is not inflated by a tick. */
            float id, iq;
            current_sense_get_foc_currents(0.0f, &id, &iq);

            step_log[n][0] = (int16_t)(ud * 1000.0f);
            step_log[n][1] = (int16_t)(id * 1000.0f);
            step_log[n][2] = (int16_t)(iq * 1000.0f);
            step_log[n][3] = (int16_t)step_sense_slack();

            if (++cnt >= STEP_HALF_TICKS) {
                cnt  = 0;
                high = !high;
                ud   = high ? STEP_V_HIGH : STEP_V_LOW;
            }
            bldc_motor_set_phase_voltage(&motor, 0.0f, ud, 0.0f);
        }
    }

    int   edges = 0;
    float i_lo  = step_ensemble(avg, (int16_t)((STEP_V_LOW + STEP_V_HIGH) * 500.0f),
                                1, &edges);
    float r_meas = motor.phase_resistance;
    float l_meas = motor.phase_inductance;

    if (edges > 0) {
        float i_hi = 0.0f;
        int   tail = (int)(STEP_HALF_TICKS * 6u / 10u);
        for (int k = tail; k < (int)STEP_HALF_TICKS; k++) i_hi += avg[k];
        i_hi /= (float)((int)STEP_HALF_TICKS - tail);

        float di_ma = i_hi - i_lo;

        /* At electrical angle 0 the phase-to-neutral voltages are
         * (ud, -ud/2, -ud/2) and the Clarke/Park chain gives i_d = i_a, so
         * i_d = ud / R_phase exactly - no scaling factors to get wrong. */
        if (di_ma > 20.0f) {
            r_meas = (STEP_V_HIGH - STEP_V_LOW) / (di_ma * 1e-3f);

            /* 63.2% crossing of the averaged step, linearly interpolated
             * between samples (tau is only ~4 samples at 200 us). */
            float target = i_lo + 0.632f * di_ma;
            float tau    = 0.0f;
            for (int k = 1; k < (int)STEP_HALF_TICKS; k++) {
                if (avg[k] >= target) {
                    float frac = (avg[k] - avg[k - 1]) > 1e-6f
                               ? (target - avg[k - 1]) / (avg[k] - avg[k - 1]) : 0.0f;
                    tau = ((float)(k - 1) + frac) / (float)HAPTIC_LOOP_HZ;
                    break;
                }
            }
            if (tau > 0.0f) l_meas = tau * r_meas;

            printk("PASS A (open loop, %d edges averaged):\n", edges);
            printk("  dI=%d mA for dU=%d mV -> R = %d mOhm\n",
                   (int)di_ma, (int)((STEP_V_HIGH - STEP_V_LOW) * 1000.0f),
                   (int)(r_meas * 1000.0f));
            printk("  tau = %d us -> L = %d uH\n",
                   (int)(tau * 1e6f), (int)(l_meas * 1e6f));
        } else {
            printk("PASS A: step too small (dI=%d mA) - rotor not locked or "
                   "current sense dead. R/L NOT updated.\n", (int)di_ma);
        }
    } else {
        printk("PASS A: no edges found in capture - timing problem.\n");
    }
    printk("  min sense slack = %d ticks (negative = shunt sampled during "
           "high-side conduction)\n", step_min_slack());

    /* ---------------- Retune from the measured parameters ---------------- */
    motor.phase_resistance = r_meas;
    motor.phase_inductance = l_meas;

    /* Cap the PI output at the largest vector both shunts can still be
     * measured at. duty_max = 0.5 + (sqrt(3)/2)*|U|/Vbus must stay under
     * 1 - 185/640 = 0.711, so |U| <= 0.211*Vbus/0.866. Beyond that the
     * measurement silently turns to noise and the loop chases its own tail. */
    float u_sense_max = (0.7109f - 0.5f) * SUPPLY_VOLTAGE / 0.866f;
    motor.voltage_limit = u_sense_max;
    motor.current_limit = u_sense_max / r_meas;

    bldc_motor_configure_current_loop(&motor, HAPTIC_CURRENT_LOOP_BW_HZ,
                                      (float)HAPTIC_LOOP_HZ,
                                      HAPTIC_CURRENT_LPF_CUTOFF_HZ);
    printk("RETUNED from measurement: Vq limit = %d mV, current limit = %d mA\n",
           (int)(u_sense_max * 1000.0f), (int)(motor.current_limit * 1000.0f));

    /* ---------------- Pass B: closed-loop d-current step ----------------- */
    pid_controller_reset(&motor.pid_current_d);
    {
        uint32_t cnt  = 0;
        bool     high = false;
        float    sp   = STEP_I_LOW;

        for (int n = 0; n < STEP_SAMPLES; n++) {
            k_sem_take(&haptic_sem, K_FOREVER);

            float id, iq;
            current_sense_get_foc_currents(0.0f, &id, &iq);
            float id_f = lowpass_filter_operator(&motor.lpf_current_d, id);
            float ud   = pid_controller_operator(&motor.pid_current_d, sp - id_f);
            bldc_motor_set_phase_voltage(&motor, 0.0f, ud, 0.0f);

            step_log[n][0] = (int16_t)(sp * 1000.0f);
            step_log[n][1] = (int16_t)(id_f * 1000.0f);
            step_log[n][2] = (int16_t)(ud * 1000.0f);
            step_log[n][3] = (int16_t)step_sense_slack();

            if (++cnt >= STEP_HALF_TICKS) {
                cnt  = 0;
                high = !high;
                sp   = high ? STEP_I_HIGH : STEP_I_LOW;
            }
        }
    }

    /* PASS B leaves pid_current_d wound (~Ud for 250 mA) and lpf_current_d
     * at ~250 mA. SVPWM of (0,0) is 50% on all phases = windings shorted.
     * Either leftover would ring the 28 Hz cogging spring. Coast Hi-Z. */
    bldc_motor_reset_current_loop_state(&motor);
    bldc_motor_set_bridge_hiz(&motor, true);

    i_lo = step_ensemble(avg, (int16_t)((STEP_I_LOW + STEP_I_HIGH) * 500.0f), 1, &edges);
    if (edges > 0) {
        float i_hi = 0.0f, peak = avg[0];
        int   tail = (int)(STEP_HALF_TICKS * 6u / 10u);
        for (int k = tail; k < (int)STEP_HALF_TICKS; k++) i_hi += avg[k];
        i_hi /= (float)((int)STEP_HALF_TICKS - tail);
        for (int k = 0; k < (int)STEP_HALF_TICKS; k++) if (avg[k] > peak) peak = avg[k];

        float span      = i_hi - i_lo;
        float overshoot = (span > 1.0f) ? ((peak - i_hi) / span * 100.0f) : 0.0f;

        int rise = 0;
        for (int k = 0; k < (int)STEP_HALF_TICKS; k++) {
            if (avg[k] >= i_lo + 0.9f * span) { rise = k; break; }
        }

        printk("PASS B (closed loop, %d edges averaged):\n", edges);
        printk("  cmd %d -> %d mA, settled %d mA, rise(90%%) %d us, overshoot %d %%\n",
               (int)(STEP_I_LOW * 1000.0f), (int)(STEP_I_HIGH * 1000.0f),
               (int)i_hi, (int)((float)rise / (float)HAPTIC_LOOP_HZ * 1e6f),
               (int)overshoot);
    }
    printk("  min sense slack = %d ticks\n", step_min_slack());
    printk("=== END LOCKED-ROTOR TEST ===\n\n");
    k_msleep(200);
}
#endif /* FOC_STEP_TEST */

/* =========================================================================
 * STANDALONE MAIN FOR TESTING
 * ========================================================================= */
int main(void)
{
    k_msleep(500);

    LOG_INF("===========================================");
    LOG_INF(" STARTING STANDALONE FOC CURRENT TEST");
    LOG_INF("===========================================");

    /* Call the initialization function defined above */
    int err = standalone_haptic_init();
    
    if (err < 0) {
        LOG_ERR("Test failed to initialize!");
        return err;
    }

    /* Everything is running in the 5kHz haptic thread.
     * We can put the main thread to sleep forever to save resources. */
    k_sleep(K_FOREVER);
    
    return 0;
}
