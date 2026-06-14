#ifndef HAPTIC_H
#define HAPTIC_H

#include <stdint.h>
#include <stdbool.h>
#include "drivers/bldc_motor.h"
#include "drivers/bldc_driver_3pwm.h"

int haptic_init(bldc_motor_t *motor, bldc_driver_t *driver, sensor_t *encoder);
void haptic_loop(bldc_motor_t *motor);
int haptic_update_num_steps_from_button(void);
int haptic_get_num_steps(void);
void haptic_set_num_steps(int steps);
void haptic_set_step_callback(void (*cb)(int dir));
void haptic_set_virtual_click_callback(void (*cb)(int dir));

/* Diagnostic line buffer:
 * haptic_init() captures diagnostic output (e.g. the open-loop torque probe)
 * into a small buffer instead of logging it directly, because bursts of log
 * lines flood the USB-CDC console and get dropped. The caller's slow main loop
 * should drain these ONE LINE PER ITERATION via haptic_diag_get_line(), which
 * matches the cadence at which USB-CDC reliably flushes.
 */
int  haptic_diag_count(void);
const char *haptic_diag_get_line(int idx);

#endif /* HAPTIC_H */
