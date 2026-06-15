/**
 * @file tmag5170_sensor.h
 * @brief TMAG5170 rotary sensor driver – FOC sensor interface
 *
 * Wraps the Zephyr TMAG5170 sensor driver and exposes the sensor_t
 * interface expected by the FOC / haptic library.
 */

#ifndef TMAG5170_SENSOR_H_
#define TMAG5170_SENSOR_H_

#include <stdbool.h>
#include "bldc_motor.h"  /* sensor_t, forward declarations */

/**
 * @brief TMAG5170 device handle.
 *
 * Kept minimal – the actual hardware is accessed via the Zephyr device
 * tree node @c tmag5170, not through a manually-managed SPI spec.
 */
struct tmag5170_device {
	bool initialized;
};

/**
 * @brief Initialize TMAG5170 encoder.
 *
 * Checks that the Zephyr TMAG5170 driver is ready.
 *
 * @param dev Pointer to @c tmag5170_device handle.
 * @return 0 on success, negative errno on failure.
 */
int tmag5170_init(struct tmag5170_device *dev);

/**
 * @brief Put the TMAG5170 into deep-sleep (low-power) mode.
 *
 * Writes the OPERATING_MODE field of the DEVICE_CONFIG register to deep-sleep
 * via the Zephyr PM device API. Used by the controller power-off sequence.
 *
 * @return 0 on success, negative errno on failure.
 */
int tmag5170_sleep(void);

/* -------------------------------------------------------------------
 * FOC sensor interface – implemented in tmag5170_sensor.c,
 * called by haptic.c / bldc_motor.c through the sensor_t abstraction.
 * ------------------------------------------------------------------- */

/** Update cached angle from TMAG5170 (rate-limited to 500 Hz). */
void sensor_update(sensor_t *sensor);

/** Return the most recently cached mechanical angle [rad]. */
float sensor_get_angle(sensor_t *sensor);

/** Return instantaneous velocity [rad/s] (stub, returns 0). */
float sensor_get_velocity(sensor_t *sensor);

/** Absolute encoder – no index search required. */
bool sensor_needs_search(sensor_t *sensor);

#endif /* TMAG5170_SENSOR_H_ */
