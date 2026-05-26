#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmag_test, LOG_LEVEL_INF);

#define TMAG_NODE DT_NODELABEL(tmag5170)

#if !DT_NODE_HAS_STATUS(TMAG_NODE, okay)
#error "TMAG5170 node is not okay in devicetree"
#endif

int main(void)
{
    const struct device *const tmag = DEVICE_DT_GET(TMAG_NODE);
    const struct device *const spi_bus = DEVICE_DT_GET(DT_BUS(TMAG_NODE));

    LOG_INF("TMAG5170 test start");
    LOG_INF("TMAG node status: %s", device_is_ready(tmag) ? "ready" : "not ready");
    LOG_INF("SPI bus for TMAG: %s (%s)", spi_bus->name,
            device_is_ready(spi_bus) ? "ready" : "not ready");

    while (1) {
        struct sensor_value angle;

        if (!device_is_ready(tmag)) {
            LOG_ERR("TMAG5170 device not ready");
            k_sleep(K_SECONDS(1));
            continue;
        }

        int ret = sensor_sample_fetch(tmag);

        if (ret == 0) {
            ret = sensor_channel_get(tmag, SENSOR_CHAN_ROTATION, &angle);
        }

        if (ret < 0) {
            LOG_ERR("TMAG read failed: %d", ret);
        } else {
            LOG_INF("angle: %d.%06d deg", angle.val1, angle.val2);
        }

        k_sleep(K_SECONDS(1));
    }

    return 0;
}
