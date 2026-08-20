/*
 * anticogging.h
 * Feedforward LUT for compensating physical motor cogging.
 */

#ifndef ANTICOGGING_H
#define ANTICOGGING_H

#include "drivers/bldc_motor.h"
#include <zephyr/kernel.h>

/* Externí deklarace tabulky a úhlu */
extern const float anticogging_lut[500];
//extern const float zero_electric_angle;

/**
 * @brief Returns the feedforward current required to overcome cogging at a given angle.
 * * @param mechanical_angle Current mechanical angle of the motor in radians.
 * @return float Current (Amperes) to be added to the Q-axis.
 */
float anticogging_get_feedforward(float mechanical_angle);

/**
 * @brief Runs the iterative anticogging calibration process.
 * This function blocks the thread until calibration is finished.
 * * @param motor Pointer to the initialized motor instance.
 * @param foc_sem Pointer to the 5kHz timing semaphore (e.g., haptic_sem).
 */
void anticogging_calibrate(bldc_motor_t *motor, struct k_sem *foc_sem);

/**
 * @brief Persist the measured FOC zero electric angle to flash.
 * @return 0 on success, negative errno on failure.
 */
int anticogging_flash_save_zero_angle(float angle);

/**
 * @brief Load the previously stored FOC zero electric angle from flash.
 * @return 0 on success, -ENOENT if nothing stored, negative errno on failure.
 */
int anticogging_flash_load_zero_angle(float *angle);

#endif /* ANTICOGGING_H */