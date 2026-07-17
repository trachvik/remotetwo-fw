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

#define FOC_DO_CALIBRATION 1

#define SUPPLY_VOLTAGE 3.75f
#define HAPTIC_VOLTAGE_LIMIT SUPPLY_VOLTAGE

/* FOC loop rate [Hz]; period = 200 µs */
#define HAPTIC_LOOP_HZ 5000U 

/* Motor parameters */
#define MOTOR_POLE_PAIRS       11
#define MOTOR_PHASE_RESISTANCE 5.6f    /* Ω */
#define MOTOR_KV_RATING        141.0f  /* rpm/V */
#define MOTOR_INDUCTANCE       0.0001f /* H = 100 µH */

/* Current loop tuning */
#define HAPTIC_CURRENT_LOOP_BW_HZ    15.0f  /* Hz; | 2 % of the foc loop frequency */
#define HAPTIC_CURRENT_LPF_CUTOFF_HZ HAPTIC_CURRENT_LOOP_BW_HZ*3.3f /* Hz; from SimpleFOC docs | 5.0 -> 3.3*/
#define HAPTIC_CURRENT_LIMIT         0.5f  /* A, hard clamp on q-current setpoint for safe bring-up */

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

    /* 2. RYCHLOSTNÍ SMYČKA - DOWNSAMPLING na 100 Hz */
    static int move_divider = 0;
    if (++move_divider >= 50) { 
        move_divider = 0;
        float target_velocity_rad_s = 6.283f; 
        bldc_motor_move(m, target_velocity_rad_s);
    }

    /* 3. ANTICOGGING INJEKCE */
    float ff = anticogging_get_feedforward(sensor_get_angle(m->sensor));
    float original_sp = m->current_sp; /* Uložíme čistou hodnotu z PIDu */
    m->current_sp += ff;               /* Přidáme mapu na překonání zubu */

    /* 4. PROUDOVÁ SMYČKA */
    bldc_motor_loop_foc(m);

    /* Obnovíme čistou PID hodnotu, aby se nezapsala zpět do struktury */
    m->current_sp = original_sp; 

    /* --- SERIALPLOT LOGOVÁNÍ (100 Hz) --- */
    static int log_counter = 0;
    if (++log_counter >= 50) { 
        log_counter = 0;
        float raw_vel = sensor_get_velocity(m->sensor);
        
        printk("DATA:%f,%f,%f\n", 
               (double)raw_vel,             
               (double)m->shaft_velocity,   
               (double)(original_sp * 10.0f)); 
               /* Vykreslujeme original_sp! V SerialPlotu byste měl vidět, jak se 
                * zelená vlna postupně zmenšuje a rovná k nule, protože se o překonávání 
                * zubů už nestará regulátor, ale mapa (ff). */
    }
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
    /* Full SimpleFOC alignSensor sequence, internal.
     * Direction and zero-electric-angle are determined by the calibration
     * itself, so nothing is pre-set here - the code is robust against swapped
     * motor phases. */
    LOG_INF("4. Running FOC calibration...");
    k_msleep(1000);
    bldc_motor_init_foc(&motor);
    LOG_INF("  [OK] FOC calibration complete!");
#else
LOG_INF("4. Bypassing calibration, setting TRUE diagnostic zero...");
    motor.sensor_direction = DIR_CCW; 
    motor.zero_electric_angle = 3.45f;
    bldc_motor_init_foc(&motor); 
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

        /* ADDED: reduce Ki to avoid integrator windup during testing */
        motor.pid_current_q.i = 20.0f; // Trochu oživíme vnitřní smyčku
        motor.pid_current_d.i = 20.0f;
        
        /* ADDED: Significant P gain to get a strong response for testing */
        motor.pid_current_q.p = 0.02f; // Zrychlíme reakci na požadavek
        motor.pid_current_d.p = 0.02f;

        // ------------- NOVÉ: RYCHLOSTNÍ SMYČKA -------------
        motor.pid_velocity.p = 0.02f;     // Mírně silnější reakce
        motor.pid_velocity.i = 0.2f;      
        motor.pid_velocity.limit = 0.25f; // Povolíme mu víc svalů na překonání zubu
        
        /* Low-pass filtr pro vyhlazení brutálního šumu ze senzoru */
        motor.lpf_velocity.tf = 0.05f;    // Silnější filtrace šumu
        motor.lpf_velocity.Ts = 0.01f;    // OPRAVA 1: Natvrdo nastavíme časový krok (100 Hz = 10 ms)
        
        /* Změna řídicího režimu: Vnější je rychlost, vnitřní je proud */
        motor.controller = VELOCITY; 
        motor.torque_controller = FOC_CURRENT;

        /* OPRAVA 2: Natvrdo vnutíme směr, aby násobič rychlosti nebyl nula! */
        motor.sensor_direction = DIR_CW;  // (Pokud se točí špatně, změníme na DIR_CCW)
        // ---------------------------------------------------

        #if FOC_DO_CALIBRATION
            bldc_motor_align_current_sense(&motor);
        #else
            current_sense_set_gain_sign(-1.0f);
        #endif
        /* FORCE CLOSED LOOP CURRENT MODE */
        motor.torque_controller = FOC_CURRENT;
        printk("  [OK] FOC_CURRENT active!");
    } else {
        LOG_ERR("  [FAIL] SAADC not ready, cannot test current control!");
        return -1; /* Stop execution if ADC is dead, no point in continuing */
    }

    printk("---> [DEBUG 2] Spoustim 5kHz casovac...\n");
    k_timer_init(&haptic_timer, haptic_timer_cb, NULL);
    k_timer_start(&haptic_timer, K_USEC(1000000U / HAPTIC_LOOP_HZ), K_USEC(1000000U / HAPTIC_LOOP_HZ));

    printk("---> [DEBUG 3] Jdu volat kalibraci...\n");
    k_msleep(10); /* Dáme procesoru 10 ms pauzu, aby si srovnal dech */

    anticogging_calibrate(&motor, &haptic_sem);

    printk("---> [DEBUG 4] Kalibrace hotova, vytvarim vlakno...\n");
    
    k_thread_create(&haptic_thread_data, haptic_stack, K_THREAD_STACK_SIZEOF(haptic_stack),
                    haptic_thread_fn, NULL, NULL, NULL, HAPTIC_THREAD_PRIORITY, K_FP_REGS, K_NO_WAIT);
    k_thread_name_set(&haptic_thread_data, "haptic_foc");

    return 0;
}

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