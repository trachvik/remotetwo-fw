/*
 * TMAG5170 encoder wrapper for FOC sensor interface.
 *
 * Implements sensor_update() / sensor_get_angle() / sensor_get_velocity() /
 * sensor_needs_search() using the Zephyr built-in ti,tmag5170 driver via the
 * standard sensor API (sensor_sample_fetch + sensor_channel_get).
 *
 * The TMAG5170 angle register returns 0–360 degrees (SENSOR_CHAN_ROTATION).
 * haptic.c expects angle in radians.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "bldc_motor.h"

LOG_MODULE_REGISTER(tmag5170_enc, LOG_LEVEL_DBG);

#define TMAG5170_NODE DT_NODELABEL(tmag5170)

static const struct device *g_tmag5170 = DEVICE_DT_GET(TMAG5170_NODE);
static float g_cached_angle_rad = 0.0f;

void sensor_update(sensor_t *sensor)
{
    ARG_UNUSED(sensor);

    if (!device_is_ready(g_tmag5170)) {
        LOG_ERR("TMAG5170 device not ready");
        return;
    }

    int ret = sensor_sample_fetch(g_tmag5170);
    if (ret < 0) {
        LOG_ERR("TMAG5170 sample fetch failed: %d", ret);
        return;
    }

    struct sensor_value val;
    ret = sensor_channel_get(g_tmag5170, SENSOR_CHAN_ROTATION, &val);
    if (ret < 0) {
        LOG_ERR("TMAG5170 channel get failed: %d", ret);
        return;
    }

    float deg = sensor_value_to_float(&val);
    g_cached_angle_rad = deg * _PI / 180.0f;
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
