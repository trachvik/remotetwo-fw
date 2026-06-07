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

	/* ---- startup diagnostic: verify SPI comms + magnet strength ----
	 * The angle noise of a Hall angle sensor scales ~ 1 / |B_xy|, where
	 * B_xy = sqrt(Bx^2 + By^2) is the IN-PLANE field magnitude that rotates
	 * with the magnet. This magnitude MUST be roughly constant as the rotor
	 * turns. Healthy diametric NdFeB Ø6x3mm @1.5mm => ~20-60 mT.
	 *   - |B_xy| only a few mT  => weak/mis-centered magnet, OR magnet is
	 *     AXIALLY magnetized (wrong type for XY angle mode) => high noise.
	 *   - |B_xy| ~0 but Bz large => definitely an axial magnet (wrong type).
	 * Sample 8 times so you can also eyeball the standstill spread.
	 */
	for (int s = 0; s < 8; s++) {
		struct sensor_value bx, by, rot;
		int rx = 0, ry = 0, rr = 0;

		if (sensor_sample_fetch(tmag) == 0) {
			rx = sensor_channel_get(tmag, SENSOR_CHAN_MAGN_X, &bx);
			ry = sensor_channel_get(tmag, SENSOR_CHAN_MAGN_Y, &by);
			rr = sensor_channel_get(tmag, SENSOR_CHAN_ROTATION, &rot);
		}

		if (rx == 0 && ry == 0 && rr == 0) {
			double dbx = sensor_value_to_double(&bx);   /* gauss */
			double dby = sensor_value_to_double(&by);   /* gauss */
			double mag_g = sqrt(dbx * dbx + dby * dby);  /* gauss */
			LOG_INF("TMAG diag[%d]: Bx=%7.3f By=%7.3f |Bxy|=%6.3f mT  angle=%8.3f deg",
				s, dbx / 10.0, dby / 10.0, mag_g / 10.0,
				sensor_value_to_double(&rot));
		} else {
			LOG_ERR("TMAG diag[%d] failed (rx=%d ry=%d rr=%d) (0 gauss => MISO stuck low)",
				s, rx, ry, rr);
		}
		k_msleep(2);
	}
	LOG_INF("TMAG diag: |Bxy| should be ~20-60 mT for a diametric Ø6x3mm @1.5mm.");
	LOG_INF("TMAG diag: a few mT => weak/decentered/AXIAL magnet (HW limit on noise).");
	/* ---------------------------------------------- */

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
	int ret;

	ret = sensor_sample_fetch(tmag);
	if (ret != 0) {
		LOG_ERR("TMAG fetch failed: %d", ret);
		return;
	}

	ret = sensor_channel_get(tmag, SENSOR_CHAN_ROTATION, &val);
	if (ret != 0) {
		LOG_ERR("TMAG channel_get failed: %d", ret);
		return;
	}

	double deg = sensor_value_to_double(&val);
	g_cached_angle_rad = (float)(deg * (M_PI / 180.0));
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
