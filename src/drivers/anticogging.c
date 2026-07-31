/*
 * anticogging.c
 * Iterative Anticogging Calibration algorithm.
 */

#include "anticogging.h"
#include "drivers/bldc_motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <math.h>

/* =========================================================================
 * OPRAVA CHYBY KOMPILÁTORU (ABI Mismatch):
 * ========================================================================= */
extern float sensor_get_angle(void *sensor);
extern void sensor_update(void *sensor);

LOG_MODULE_REGISTER(anticogging, LOG_LEVEL_INF);

#define SAMPLE_COUNT 500
#define TWO_PI 6.28318530718f
#define CALIB_SPEED_RAD_S 3.14159f
#define CALIB_REVOLUTIONS 3
#define MAX_ITERATIONS 4
#define RELAXATION_GAIN 0.8f 
#define ERROR_THRESHOLD 0.005f 

static float final_lut[SAMPLE_COUNT] = {0};

/* --- SAFE MATH ---
 * Protects against HardFault, NaN values, and infinite loops. */
static inline int get_lut_index(float mechanical_angle) {
    if (isnan(mechanical_angle) || isinf(mechanical_angle)) {
        return 0;
    }
    
    float norm_angle = fmodf(mechanical_angle, TWO_PI);
    if (norm_angle < 0.0f) {
        norm_angle += TWO_PI;
    }
    
    int index = (int)((norm_angle / TWO_PI) * SAMPLE_COUNT);
    
    /* Hard safety limits */
    if (index < 0) index = 0;
    if (index >= SAMPLE_COUNT) index = SAMPLE_COUNT - 1;
    
    return index;
}

float anticogging_get_feedforward(float mechanical_angle) 
{
    return final_lut[get_lut_index(mechanical_angle)];
}

static void run_directional_calibration(bldc_motor_t *motor, struct k_sem *foc_sem, float target_speed, float *output_lut) 
{
    static float temp_sum[SAMPLE_COUNT];
    static uint16_t temp_count[SAMPLE_COUNT];

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        temp_sum[i] = 0.0f;
        temp_count[i] = 0;
    }

    uint32_t stabilize_ticks = (uint32_t)(2.0f * 5000.0f); 
    int move_divider = 0;
    
    printk("     -> Stabilizacni faze (2s)...\n");

    for(uint32_t i = 0; i < stabilize_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 50) {
            move_divider = 0;
            bldc_motor_move(motor, target_speed);
        }
        
        float angle = sensor_get_angle(motor->sensor);
        float ff = anticogging_get_feedforward(angle);
        
        float original_sp = motor->current_sp;
        motor->current_sp += ff; 
        
        bldc_motor_loop_foc(motor);
        
        motor->current_sp = original_sp; 

        /* LOGOVANI STABILIZACE (1 Hz) */
        if (i % 5000 == 0) {
            printk("       [1s STATUS] Stabilizace | Aktualni uhel: %8.4f rad\n", (double)angle);
        }
    }

    printk("     -> Sber dat spusteni...\n");

    /* 2. Data collection phase */
    float time_needed_s = (CALIB_REVOLUTIONS * TWO_PI) / fabs(target_speed);
    uint32_t calib_ticks = (uint32_t)(time_needed_s * 5000.0f);

    for(uint32_t i = 0; i < calib_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 50) {
            move_divider = 0;
            bldc_motor_move(motor, target_speed);
        }

        float angle = sensor_get_angle(motor->sensor);
        float ff = anticogging_get_feedforward(angle);
        
        float original_sp = motor->current_sp;
        motor->current_sp += ff; 
        bldc_motor_loop_foc(motor);
        motor->current_sp = original_sp; 

        int index = get_lut_index(angle);
        temp_sum[index] += original_sp;
        temp_count[index]++;

        /* LOGOVANI SBERU DAT (1 Hz) */
        if (i % 5000 == 0) {
            printk("       [1s STATUS] Sber dat | Uhel: %8.4f rad | Zapisuji do indexu: [%3d/%d] | Proud z PID: %6.3f A\n", 
                   (double)angle, index, SAMPLE_COUNT, (double)original_sp);
        }
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
    static float lut_cw[SAMPLE_COUNT];
    static float lut_ccw[SAMPLE_COUNT];

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        final_lut[i] = 0.0f;
    }
    
    printk("---> PUVODNI ITERATIVNI KALIBRACE START\n");

    for (int iter = 1; iter <= MAX_ITERATIONS; iter++) {
        printk("---> ITERATION %d / %d ---\n", iter, MAX_ITERATIONS);

        run_directional_calibration(motor, foc_sem, CALIB_SPEED_RAD_S, lut_cw);
        k_msleep(500); 
        run_directional_calibration(motor, foc_sem, -CALIB_SPEED_RAD_S, lut_ccw);

        float max_residual_error = 0.0f;
        float sum_residual_error = 0.0f;

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float avg_residual = (lut_cw[i] + lut_ccw[i]) / 2.0f;
            final_lut[i] += (avg_residual * RELAXATION_GAIN);

            float abs_err = fabs(avg_residual);
            sum_residual_error += abs_err;
            if (abs_err > max_residual_error) max_residual_error = abs_err;
        }

        float mean_error = sum_residual_error / SAMPLE_COUNT;
        printk("---> End of iteration %d: Max error = %.4f A\n", iter, (double)max_residual_error);

        if (max_residual_error < ERROR_THRESHOLD) {
            printk("---> Error is below threshold. Calibration finished early!\n");
            break; 
        }
        k_msleep(1000); 
    }

    printk("---> CALIBRATION COMPLETED ---\n");

    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
}