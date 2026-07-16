/*
 * anticogging.h
 * Feedforward LUT for compensating physical motor cogging.
 */

#ifndef ANTICOGGING_H
#define ANTICOGGING_H

#include "drivers/bldc_motor.h"
#include <zephyr/kernel.h>

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

#endif /* ANTICOGGING_H */