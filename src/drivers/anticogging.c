#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

// TO DO : replace functions with the ones from the FOC library (foc_get_mechanical_angle, foc_get_pi_iq_output, etc.)

LOG_MODULE_REGISTER(haptic_calib, LOG_LEVEL_INF);

#define SAMPLE_COUNT 2000
#define TWO_PI 6.28318530718f
#define CALIB_SPEED_RAD_S 1.0f 
#define CALIB_REVOLUTIONS 3

// --- ITERATION SETTINGS ---
#define MAX_ITERATIONS 5
#define RELAXATION_GAIN 0.8f // Add only 80% of the measured error for learning stability
#define ERROR_THRESHOLD 0.01f // End calibration if the error is less than 10 mA // TO DO : adjust this threshold

float final_lut[SAMPLE_COUNT] = {0};

extern void foc_set_velocity_target(float rad_s);
extern float foc_get_mechanical_angle(void); 
extern float foc_get_pi_iq_output(void); // WARNING: Must return only the pure effort of the velocity PI controller!
extern void foc_loop_step(void);

// Helper function remains mostly the same, just uses foc_get_pi_iq_output
static void run_directional_calibration(float speed, float *output_lut) 
{
    static float temp_sum[SAMPLE_COUNT];
    static uint16_t temp_count[SAMPLE_COUNT];

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        temp_sum[i] = 0.0f;
        temp_count[i] = 0;
    }

    // motor.controller = MotionControlType::velocity_openloop; | SimpleFOC
    foc_set_velocity_target(speed);

    // Stabilization
    uint32_t stabilize_start = k_uptime_get_32();
    while (k_uptime_get_32() - stabilize_start < 2000) {
        foc_loop_step();
        k_busy_wait(200); 
    }

    float time_needed_s = (CALIB_REVOLUTIONS * TWO_PI) / fabs(speed);
    uint32_t duration_ms = (uint32_t)(time_needed_s * 1000.0f);

    // Data collection
    uint32_t start_time = k_uptime_get_32();
    while (k_uptime_get_32() - start_time < duration_ms) {
        
        float angle = foc_get_mechanical_angle();
        // Read only the residual error not resolved by the Feedforward from final_lut
        float residual_iq = foc_get_pi_iq_output(); // It is important not to read the compensated value, but the pure current value from the PID

        while (angle < 0.0f) angle += TWO_PI;
        while (angle >= TWO_PI) angle -= TWO_PI;

        int index = (int)((angle / TWO_PI) * SAMPLE_COUNT);
        if (index >= SAMPLE_COUNT) index = SAMPLE_COUNT - 1; 

        temp_sum[index] += residual_iq;
        temp_count[index]++;

        foc_loop_step(); // Inside this step, FOC reads and applies final_lut[index]!
        k_busy_wait(200); 
    }

    foc_set_velocity_target(0.0f);

    // Calculation of the average residual error for the given direction
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (temp_count[i] > 0) {
            output_lut[i] = temp_sum[i] / (float)temp_count[i];
        } else {
            output_lut[i] = (i > 0) ? output_lut[i - 1] : 0.0f;
        }
    }
}

/**
 * @brief Main iterative loop for calibration
 */
void calibrate_anticogging_iterative(void) 
{
    LOG_INF("--- START OF ITERATIVE CALIBRATION ---");

    static float lut_cw[SAMPLE_COUNT];
    static float lut_ccw[SAMPLE_COUNT];

    // Zeroing out the previous map just to be sure
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = 0.0f;

    for (int iter = 1; iter <= MAX_ITERATIONS; iter++) {
        LOG_INF("--- ITERATION %d / %d ---", iter, MAX_ITERATIONS);

        run_directional_calibration(CALIB_SPEED_RAD_S, lut_cw);
        k_msleep(500); 
        run_directional_calibration(-CALIB_SPEED_RAD_S, lut_ccw);

        // Processing of residual error and updating final_lut
        float max_residual_error = 0.0f;
        float sum_residual_error = 0.0f;

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            // Average the residual error from both directions (latency removal)
            float avg_residual = (lut_cw[i] + lut_ccw[i]) / 2.0f;
            
            // Add the error to our main map (multiplied by the relaxation gain)
            final_lut[i] += (avg_residual * RELAXATION_GAIN);

            // Statistics for evaluation
            float abs_err = fabs(avg_residual);
            sum_residual_error += abs_err;
            if (abs_err > max_residual_error) {
                max_residual_error = abs_err;
            }
        }

        float mean_error = sum_residual_error / SAMPLE_COUNT;
        LOG_INF("End of iteration %d: Max error = %.4f A, Average error = %.4f A", 
                iter, (double)max_residual_error, (double)mean_error);

        // Dynamic termination of calibration if the error is already negligible
        if (max_residual_error < ERROR_THRESHOLD) {
            LOG_INF("Error is below the limit %.4f A. Calibration successfully finished early!", (double)ERROR_THRESHOLD);
            break; 
        }

        k_msleep(1000); // Pause before the next iteration (let the motor cool down, etc.)
    }

    LOG_INF("--- CALIBRATION COMPLETED ---");
}