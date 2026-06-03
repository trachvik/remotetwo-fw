/**
 * @file tmag5170_sensor.c
 * @brief TMAG5170 rotary sensor driver – FOC sensor interface
 *
 * Uses the Zephyr TMAG5170 driver (CONFIG_TMAG5170=y) via the standard
 * sensor API.  The FOC interface (sensor_update / sensor_get_angle …)
 * is rate-limited to 500 Hz to avoid SPI contention with the SSD1309
 * display driver sharing the same SPI0 bus.
 */

#include "tmag5170_sensor.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(tmag5170_sensor, LOG_LEVEL_INF);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 500 Hz rate limit – update at most once every 2 ms */
#define SENSOR_UPDATE_MIN_INTERVAL_US  2000

static float   g_cached_angle_rad  = 0.0f;
static int64_t g_last_update_us    = 0;

/* ------------------------------------------------------------------ */

int tmag5170_init(struct tmag5170_device *dev)
{
	if (!dev) {
		return -EINVAL;
	}

	const struct device *tmag = DEVICE_DT_GET(DT_NODELABEL(tmag5170));

	if (!device_is_ready(tmag)) {
		LOG_ERR("TMAG5170 device not ready");
		return -ENODEV;
	}

	dev->initialized = true;
	LOG_INF("TMAG5170 encoder initialized");
	return 0;
}

/* ------------------------------------------------------------------ */

void sensor_update(sensor_t *sensor)
{
	ARG_UNUSED(sensor);

	/* Rate-limit to 500 Hz */
	int64_t now_us = k_ticks_to_us_near64(k_uptime_ticks());

	if ((now_us - g_last_update_us) < SENSOR_UPDATE_MIN_INTERVAL_US) {
		return;
	}
	g_last_update_us = now_us;

	const struct device *tmag = DEVICE_DT_GET(DT_NODELABEL(tmag5170));
	struct sensor_value val;

	if (sensor_sample_fetch_chan(tmag, SENSOR_CHAN_ROTATION) == 0 &&
	    sensor_channel_get(tmag, SENSOR_CHAN_ROTATION, &val) == 0) {
		double deg = sensor_value_to_double(&val);
		g_cached_angle_rad = (float)(deg * (M_PI / 180.0));
	}
}

float sensor_get_angle(sensor_t *sensor)
{
	ARG_UNUSED(sensor);
	return g_cached_angle_rad;
}

float sensor_get_velocity(sensor_t *sensor)
{
	ARG_UNUSED(sensor);
	return 0.0f;
}

bool sensor_needs_search(sensor_t *sensor)
{
	ARG_UNUSED(sensor);
	return false;
}
