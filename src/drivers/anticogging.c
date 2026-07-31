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
    
    for(uint32_t i = 0; i < stabilize_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 5) { // 1000 Hz update for motion control
            move_divider = 0;
            bldc_motor_move(motor, target_speed);
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
    static float smoothed_lut[SAMPLE_COUNT];

    float saved_kp = motor->pid_velocity.p;
    float saved_ki = motor->pid_velocity.i;
    float saved_v_limit = motor->pid_velocity.limit;
    float saved_c_limit = motor->current_limit;

    /* OPRAVA: Návrat k "měkké gumě" bez Integrator Windup efektu */
    motor->pid_velocity.p = 0.05f;       /* Hladké obkreslení zubů (žádné kmitání) */
    motor->pid_velocity.i = 0.02f;       /* Miniaturní I-složka jen proti zastavení na tření */
    motor->pid_velocity.limit = 1.0f;    
    motor->current_limit = 1.0f;     

    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = 0.0f;
    
    printk("---> ANTICOGGING CALIBRATION START\n");

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

        float mean_error = sum_residual_error / (float)SAMPLE_COUNT;
        printk("     Mean residual error = %.4f A (Max spike = %.4f A)\n", (double)mean_error, (double)max_residual_error);

        if (max_residual_error < ERROR_THRESHOLD) {
            printk("---> Error is below threshold. Calibration finished early!\n");
            break; 
        }
        k_msleep(500); 
    }

    /* =========================================================================
     * 3. FILTRACE: Aplikace vyhlazení až na zcela spojenou finální mapu
     * ========================================================================= */
    int window = 5; 
    int half_w = window / 2;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float sum = 0.0f;
        for (int j = -half_w; j <= half_w; j++) {
            int idx = (i + j + SAMPLE_COUNT) % SAMPLE_COUNT; 
            sum += final_lut[idx];
        }
        smoothed_lut[i] = sum / (float)window;
    }
    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = smoothed_lut[i];

    /* Návrat uživatelských PID hodnot po úspěšné kalibraci */
    motor->pid_velocity.p = saved_kp;
    motor->pid_velocity.i = saved_ki;
    motor->pid_velocity.limit = saved_v_limit;
    motor->current_limit = saved_c_limit;

    printk("---> CALIBRATION COMPLETED ---\n");
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
}