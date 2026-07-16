/*
 * foc_current.c
 * Standalone application for testing closed-loop current FOC control.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Adjust these includes based on your project structure */
#include "drivers/tmag5170_sensor.h"
#include "drivers/current_sense.h"
#include "drivers/bldc_motor.h"
#include "drivers/bldc_driver_3pwm.h"

LOG_MODULE_REGISTER(foc_current_test, LOG_LEVEL_INF);

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
#define HAPTIC_CURRENT_LOOP_BW_HZ    50.0f  /* Hz; stability-first baseline */
#define HAPTIC_CURRENT_LPF_CUTOFF_HZ 120.0f /* Hz; stronger noise rejection */
#define HAPTIC_CURRENT_LIMIT         0.25f  /* A, hard clamp on q-current setpoint for safe bring-up */

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
    /* 1. Update sensor (Required for electrical angle and Park/Clarke transformations) */
    sensor_update(m->sensor);

    /* =========================================================================
     * TEST ZONE: Set the target current here for testing
     * ========================================================================= 
     * 0.0f -> Silence test (motor must stand still and make no noise)
     * 0.2f -> Smooth pull test (motor pulls steadily without whining or vibrating)
     */
    float target_current_amps = 0.0f;

    /* Safety clamp */
    if (target_current_amps > HAPTIC_CURRENT_LIMIT) target_current_amps = HAPTIC_CURRENT_LIMIT;
    if (target_current_amps < -HAPTIC_CURRENT_LIMIT) target_current_amps = -HAPTIC_CURRENT_LIMIT;

    /* 2. Send to FOC algorithm */
    bldc_motor_move(m, target_current_amps);
    bldc_motor_loop_foc(m);
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
    tmag5170_init(&encoder);

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
    motor.voltage_limit = HAPTIC_VOLTAGE_LIMIT * 0.58f; 
    motor.velocity_limit = 20.0f;
    motor.voltage_sensor_align = 1.5f;
    bldc_motor_init(&motor);

    /* 4. Run FOC calibration */
    LOG_INF("4. Running FOC calibration...");
    k_msleep(1000);

    motor.sensor_direction = DIR_CW;
    motor.zero_electric_angle = 0.0f;
    
    /* Ramp up voltage to align the rotor */
    for (int i = 0; i <= 50; i++) {
        float v = motor.voltage_sensor_align * (float)i / 50.0f;
        bldc_motor_set_phase_voltage(&motor, v, 0.0f, 4.71239f);
        k_msleep(1);
    }
    k_msleep(700); /* Let the rotor settle */
    
    sensor_update(motor.sensor);
    motor.zero_electric_angle = bldc_motor_electrical_angle(&motor);
    bldc_motor_set_phase_voltage(&motor, 0.0f, 0.0f, 0.0f);
    k_msleep(100);

    bldc_motor_init_foc(&motor);
    LOG_INF("  [OK] FOC calibration complete!");

    /* 5. Initialize FOC current sense (DRV8311H CSA via SAADC) */
    LOG_INF("5. Initializing FOC current sense...");
    int cs_err = current_sense_init();
    
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

        bldc_motor_align_current_sense(&motor);
        
        /* FORCE CLOSED LOOP CURRENT MODE */
        motor.torque_controller = FOC_CURRENT;
        LOG_INF("  [OK] FOC_CURRENT active!");

        /* Warm-up: run 100 FOC iterations at 0 A before starting the timer.
         * Mirrors haptic.c's post-enable settle loop.  This gives the current
         * PID integrators time to converge to zero and lets the SAADC DPPI
         * chain reach steady-state.  Without this, bldc_motor_align_current_sense
         * leaves the DPPI/TIMER2 chain freshly armed (~0 ms ago); the first
         * sensor_sample_fetch in the haptic thread hits this noisy window and
         * returns -EIO, which self-reinforces into permanent CRC failures. */
        motor.target = 0.0f;
        for (int i = 0; i < 100; i++) {
            bldc_motor_loop_foc(&motor);
            bldc_motor_move(&motor, 0.0f);
            k_msleep(1);
        }
        k_msleep(200);

        /* Final sensor update before the 5 kHz timer fires.
         * Sets g_last_update_us = now, so the first few haptic-thread calls
         * hit the 800 µs rate limiter and skip sensor_sample_fetch.  By the
         * time the limiter opens (~800 µs later), the DPPI chain and PLL are
         * fully settled and the CRC checks pass reliably. */
        sensor_update(motor.sensor);
    } else {
        LOG_ERR("  [FAIL] SAADC not ready, cannot test current control!");
        return -1; /* Stop execution if ADC is dead, no point in continuing */
    }

    /* Start 5 kHz FOC control loop */
    k_timer_init(&haptic_timer, haptic_timer_cb, NULL);
    k_thread_create(&haptic_thread_data, haptic_stack, K_THREAD_STACK_SIZEOF(haptic_stack),
                    haptic_thread_fn, NULL, NULL, NULL, HAPTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&haptic_thread_data, "haptic_foc");
    k_timer_start(&haptic_timer, K_USEC(1000000U / HAPTIC_LOOP_HZ), K_USEC(1000000U / HAPTIC_LOOP_HZ));

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