/**
 * @file tmag5170.c
 * @brief TMAG5170 magnetic angle sensor driver using Zephyr sensor API
 */

#include "tmag5170.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(tmag5170, LOG_LEVEL_INF);

#define TMAG5170_NODE DT_NODELABEL(tmag5170)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int tmag5170_init(struct tmag5170_device *dev)
{
        if (!dev) {
                return -EINVAL;
        }

        dev->dev = DEVICE_DT_GET(TMAG5170_NODE);
        if (!device_is_ready(dev->dev)) {
                LOG_ERR("TMAG5170 device not ready");
                return -ENODEV;
        }

        LOG_INF("TMAG5170 initialized");
        return 0;
}

int tmag5170_read_angle_deg(struct tmag5170_device *dev, float *angle_deg)
{
        if (!dev || !angle_deg) {
                return -EINVAL;
        }

        int ret = sensor_sample_fetch_chan(dev->dev, SENSOR_CHAN_ROTATION);
        if (ret < 0) {
                LOG_ERR("TMAG5170 fetch failed: %d", ret);
                return ret;
        }

        struct sensor_value val;
        ret = sensor_channel_get(dev->dev, SENSOR_CHAN_ROTATION, &val);
        if (ret < 0) {
                LOG_ERR("TMAG5170 get failed: %d", ret);
                return ret;
        }

        *angle_deg = (float)val.val1 + (float)val.val2 / 1000000.0f;
        return 0;
}

int tmag5170_read_angle_rad(struct tmag5170_device *dev, float *angle_rad)
{
        float deg;
        int ret = tmag5170_read_angle_deg(dev, &deg);
        if (ret < 0) {
                return ret;
        }

        *angle_rad = deg * (float)M_PI / 180.0f;
        return 0;
}

/*
 * Sensor abstraction interface — used by bldc_motor.c / haptic.c when
 * full haptic stack is compiled.  The sensor parameter is unused; the
 * global g_tmag5170 pointer (set from main) is the actual device handle.
 */

/* Forward declaration — matches typedef struct sensor sensor_t in bldc_motor.h */
struct sensor;
typedef struct sensor sensor_t;

extern struct tmag5170_device *g_tmag5170;

static float g_cached_angle_rad = 0.0f;

void sensor_update(sensor_t *sensor)
{
        ARG_UNUSED(sensor);
        if (g_tmag5170 == NULL) {
                return;
        }
        tmag5170_read_angle_rad(g_tmag5170, &g_cached_angle_rad);
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
