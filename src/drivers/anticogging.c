/*
 * anticogging.c
 * Iterative Anticogging Calibration algorithm.
 */

#include "anticogging.h" // Update path if necessary, e.g., "drivers/anticogging.h"
#include "drivers/sensor.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(anticogging, LOG_LEVEL_INF);

#define SAMPLE_COUNT 2000
#define TWO_PI 6.28318530718f
#define CALIB_SPEED_RAD_S 1.0f 
#define CALIB_REVOLUTIONS 3

/* Iteration Settings */
#define MAX_ITERATIONS 5
#define RELAXATION_GAIN 0.8f  /* Add only 80% of the measured error for learning stability */
#define ERROR_THRESHOLD 0.01f /* End calibration if max error is less than 10 mA */

/* Internal PI controller gains for the calibration speed loop */
#define CALIB_KP 0.2f
#define CALIB_KI 2.0f
#define MAX_CALIB_CURRENT 0.5f /* Safe maximum current during calibration (Amperes) */
#define LOOP_DT 0.0002f        /* 5 kHz loop period (200 us) */

/* The main Look-Up Table storing feedforward current in Amperes */
static float final_lut[SAMPLE_COUNT] = {0};

float anticogging_get_feedforward(float mechanical_angle) 
{
    float norm_angle = mechanical_angle;
    while (norm_angle < 0.0f) norm_angle += TWO_PI;
    while (norm_angle >= TWO_PI) norm_angle -= TWO_PI;

    int index = (int)((norm_angle / TWO_PI) * SAMPLE_COUNT);
    if (index >= SAMPLE_COUNT) {
        index = SAMPLE_COUNT - 1; 
    }
    
    return final_lut[index];
}

static void run_directional_calibration(bldc_motor_t *motor, struct k_sem *foc_sem, float target_speed, float *output_lut) 
{
    static float temp_sum[SAMPLE_COUNT];
    static uint16_t temp_count[SAMPLE_COUNT];

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        temp_sum[i] = 0.0f;
        temp_count[i] = 0;
    }

    float integral = 0.0f;
    sensor_update(motor->sensor);
    float prev_angle = sensor_get_angle(motor->sensor);

    /* 1. Stabilization Phase (2 seconds) 
     * Let the PI controller smoothly accelerate the motor and stabilize the speed. */
    uint32_t stabilize_ticks = (uint32_t)(2.0f * 5000.0f); 
    
    for(uint32_t i = 0; i < stabilize_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER); /* Wait for the 5kHz timer tick */
        
        sensor_update(motor->sensor);
        float angle = sensor_get_angle(motor->sensor);
        
        /* Calculate instantaneous velocity (with wrap-around protection) */
        float d_ang = angle - prev_angle;
        while (d_ang >  3.14159f) d_ang -= TWO_PI;
        while (d_ang < -3.14159f) d_ang += TWO_PI;
        prev_angle = angle;
        
        float vel = d_ang / LOOP_DT; 

        /* Local Velocity PI Controller */
        float err = target_speed - vel;
        integral += err * CALIB_KI * LOOP_DT;
        float iq_target = (CALIB_KP * err) + integral;
        
        /* Clamp output for safety */
        if (iq_target > MAX_CALIB_CURRENT) iq_target = MAX_CALIB_CURRENT;
        if (iq_target < -MAX_CALIB_CURRENT) iq_target = -MAX_CALIB_CURRENT;

        /* Apply current to motor (Feedforward from previous iterations is added here!) */
        float ff = anticogging_get_feedforward(angle);
        bldc_motor_move(motor, iq_target + ff);
        bldc_motor_loop_foc(motor);
    }

    /* 2. Data Collection Phase */
    float time_needed_s = (CALIB_REVOLUTIONS * TWO_PI) / fabs(target_speed);
    uint32_t calib_ticks = (uint32_t)(time_needed_s * 5000.0f);

    for(uint32_t i = 0; i < calib_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        
        sensor_update(motor->sensor);
        float angle = sensor_get_angle(motor->sensor);
        
        float d_ang = angle - prev_angle;
        while (d_ang >  3.14159f) d_ang -= TWO_PI;
        while (d_ang < -3.14159f) d_ang += TWO_PI;
        prev_angle = angle;
        
        float vel = d_ang / LOOP_DT; 

        /* Local Velocity PI Controller */
        float err = target_speed - vel;
        integral += err * CALIB_KI * LOOP_DT;
        float iq_target = (CALIB_KP * err) + integral;
        
        if (iq_target > MAX_CALIB_CURRENT) iq_target = MAX_CALIB_CURRENT;
        if (iq_target < -MAX_CALIB_CURRENT) iq_target = -MAX_CALIB_CURRENT;

        float ff = anticogging_get_feedforward(angle);
        bldc_motor_move(motor, iq_target + ff);
        bldc_motor_loop_foc(motor);

        /* --- DATA LOGGING --- 
         * We only log 'iq_target' (the pure effort of our PI controller).
         * This represents the uncompensated residual cogging error. */
        float norm_angle = angle;
        while (norm_angle < 0.0f) norm_angle += TWO_PI;
        while (norm_angle >= TWO_PI) norm_angle -= TWO_PI;

        int index = (int)((norm_angle / TWO_PI) * SAMPLE_COUNT);
        if (index >= SAMPLE_COUNT) index = SAMPLE_COUNT - 1; 

        temp_sum[index] += iq_target;
        temp_count[index]++;
    }

    /* Stop the motor after calibration run */
    bldc_motor_move(motor, 0.0f);
    bldc_motor_loop_foc(motor);

    /* 3. Average the collected data into the output LUT */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (temp_count[i] > 0) {
            output_lut[i] = temp_sum[i] / (float)temp_count[i];
        } else {
            /* If a bin was skipped (due to high speed), copy the previous value */
            output_lut[i] = (i > 0) ? output_lut[i - 1] : 0.0f;
        }
    }
}

void anticogging_calibrate(bldc_motor_t *motor, struct k_sem *foc_sem) 
{
    LOG_INF("--- START OF ITERATIVE ANTICOGGING CALIBRATION ---");

    static float lut_cw[SAMPLE_COUNT];
    static float lut_ccw[SAMPLE_COUNT];

    /* Clear the main LUT before starting */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        final_lut[i] = 0.0f;
    }

    for (int iter = 1; iter <= MAX_ITERATIONS; iter++) {
        LOG_INF("--- ITERATION %d / %d ---", iter, MAX_ITERATIONS);

        /* Run forward and backward to eliminate dynamic latencies */
        run_directional_calibration(motor, foc_sem, CALIB_SPEED_RAD_S, lut_cw);
        k_msleep(500); 
        run_directional_calibration(motor, foc_sem, -CALIB_SPEED_RAD_S, lut_ccw);

        /* Process residual error and update final_lut */
        float max_residual_error = 0.0f;
        float sum_residual_error = 0.0f;

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            /* Average the residual error from both directions */
            float avg_residual = (lut_cw[i] + lut_ccw[i]) / 2.0f;
            
            /* Apply relaxation gain and add to the main LUT */
            final_lut[i] += (avg_residual * RELAXATION_GAIN);

            /* Statistics */
            float abs_err = fabs(avg_residual);
            sum_residual_error += abs_err;
            if (abs_err > max_residual_error) {
                max_residual_error = abs_err;
            }
        }

        float mean_error = sum_residual_error / SAMPLE_COUNT;
        LOG_INF("End of iteration %d: Max error = %.4f A, Average error = %.4f A", 
                iter, (double)max_residual_error, (double)mean_error);

        /* Early exit if error is negligible */
        if (max_residual_error < ERROR_THRESHOLD) {
            LOG_INF("Error is below threshold (%.4f A). Calibration finished early!", (double)ERROR_THRESHOLD);
            break; 
        }

        k_msleep(1000); /* Let the motor cool down before next iteration */
    }

    LOG_INF("--- CALIBRATION COMPLETED ---");
}