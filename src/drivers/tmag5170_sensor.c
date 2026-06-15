/**
 * @file tmag5170_sensor.c
 * @brief TMAG5170 rotary sensor driver – FOC sensor interface
 *
 * Uses the Zephyr TMAG5170 driver (CONFIG_TMAG5170=y) via the standard
 * sensor API.  The FOC interface (sensor_update / sensor_get_angle …)
 * is rate-limited to 500 Hz to avoid SPI contention with the SSD1309
 * display driver sharing the same SPIM4 (HSPI) bus.
 */

#include "tmag5170_sensor.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/crc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(tmag5170_sensor, LOG_LEVEL_INF);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 500 Hz rate limit – update at most once every 2 ms */
#define SENSOR_UPDATE_MIN_INTERVAL_US  2000

/* ---- Direct register-write deep-sleep ----------------------------------
 * CONFIG_PM_DEVICE perturbs early device init on this target (the app then
 * fails to boot), so we cannot use the upstream pm_action SUSPEND path.
 * Instead we replicate exactly what that handler does: write OPERATING_MODE
 * = DEEP_SLEEP into the DEVICE_CONFIG register over SPI.
 *
 * Frame format (matches the upstream tmag5170_write_register):
 *   byte0 = register address (0x00 = DEVICE_CONFIG)
 *   byte1 = data[15:8]
 *   byte2 = data[7:0]
 *   byte3 = CRC4 in the low nibble (CONFIG_TMAG5170_CRC=y keeps chip CRC on)
 * DEVICE_CONFIG value: OPERATING_MODE (bits 6:4) = 0x6 (DEEP_SLEEP) => 0x0060.
 */
#define TMAG_REG_DEVICE_CONFIG     0x00U
#define TMAG_OPMODE_DEEP_SLEEP     0x6U
#define TMAG_DEVICE_CONFIG_SLEEP   ((TMAG_OPMODE_DEEP_SLEEP & 0x7U) << 4)  /* 0x0060 */
#define TMAG_CRC_SEED              0xFU

static const struct spi_dt_spec g_tmag_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(tmag5170),
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
			0);

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

int tmag5170_sleep(void)
{
	const struct device *tmag = DEVICE_DT_GET(DT_NODELABEL(tmag5170));

	if (!device_is_ready(tmag)) {
		return -ENODEV;
	}

	if (!spi_is_ready_dt(&g_tmag_spi)) {
		LOG_ERR("TMAG5170 SPI bus not ready");
		return -ENODEV;
	}

	/* DEVICE_CONFIG = OPERATING_MODE(DEEP_SLEEP). Other DEVICE_CONFIG fields
	 * (CONV_AVG, MAG_TEMPCO, T_CH_EN, T_RATE) are don't-care in deep sleep. */
	uint8_t tx[4] = {
		TMAG_REG_DEVICE_CONFIG,
		(TMAG_DEVICE_CONFIG_SLEEP >> 8) & 0xFF,
		TMAG_DEVICE_CONFIG_SLEEP & 0xFF,
		0x00U,
	};

#if defined(CONFIG_TMAG5170_CRC)
	/* Chip keeps CRC enabled (init never sent the disable-CRC packet), so the
	 * frame must carry a valid CRC4 in the low nibble of the last byte. */
	tx[3] &= 0xF0U;
	tx[3] |= crc4_ti(TMAG_CRC_SEED, tx, sizeof(tx)) & 0x0FU;
#endif

	const struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	int ret = spi_write_dt(&g_tmag_spi, &tx_set);

	if (ret != 0) {
		LOG_ERR("TMAG5170 deep-sleep write failed: %d", ret);
		return ret;
	}

	LOG_INF("TMAG5170 entered deep-sleep (DEVICE_CONFIG register write)");
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
