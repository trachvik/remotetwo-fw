#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(tmag_test, LOG_LEVEL_INF);

#define TMAG_NODE DT_NODELABEL(tmag5170)

#if !DT_NODE_HAS_STATUS(TMAG_NODE, okay)
#error "TMAG5170 node is not okay in devicetree"
#endif

/* Raw SPI access for register diagnostics.
 * TMAG5170 uses pipelined SPI: RX during transaction N = response to transaction N-1.
 * Reading a register TWICE gives the actual current value on the 2nd read.
 */
static const struct spi_dt_spec tmag_spi = SPI_DT_SPEC_GET(
    TMAG_NODE,
    SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    0);

static uint16_t tmag_read_reg_raw(uint8_t reg)
{
    uint8_t tx[4] = { BIT(7) | reg, 0x00, 0x00, 0x00 };
    uint8_t rx[4] = { 0 };
    struct spi_buf tx_buf = { .buf = tx, .len = 4 };
    struct spi_buf rx_buf = { .buf = rx, .len = 4 };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    /* 1st read: flushes pipeline (gets previous command's response) */
    spi_transceive_dt(&tmag_spi, &tx_set, &rx_set);
    memset(rx, 0, sizeof(rx));
    /* 2nd read: gets actual current register value */
    spi_transceive_dt(&tmag_spi, &tx_set, &rx_set);
    return (uint16_t)((rx[1] << 8) | rx[2]);
}

int main(void)
{
    const struct device *const tmag = DEVICE_DT_GET(TMAG_NODE);
    const struct device *const spi_bus = DEVICE_DT_GET(DT_BUS(TMAG_NODE));

    LOG_INF("TMAG5170 test start");
    LOG_INF("TMAG node status: %s", device_is_ready(tmag) ? "ready" : "not ready");
    LOG_INF("SPI bus for TMAG: %s (%s)", spi_bus->name,
            device_is_ready(spi_bus) ? "ready" : "not ready");

    /* --- one-time register dump right after init ---
     * Expected: DEVICE_CONFIG=0x0020 (AMM), SENSOR_CONFIG=0x40C0 (XY+angle)
     */
    uint16_t dev_cfg   = tmag_read_reg_raw(0x00);  /* DEVICE_CONFIG  */
    uint16_t sens_cfg  = tmag_read_reg_raw(0x01);  /* SENSOR_CONFIG  */
    uint16_t test_cfg  = tmag_read_reg_raw(0x0F);  /* TEST_CONFIG    */
    LOG_INF("POST-INIT regs: DEVICE_CONFIG=0x%04x (exp 0x0020)  "
            "SENSOR_CONFIG=0x%04x (exp 0x40C0)  TEST_CONFIG=0x%04x",
            dev_cfg, sens_cfg, test_cfg);

    while (1) {
        struct sensor_value angle;

        if (!device_is_ready(tmag)) {
            LOG_ERR("TMAG5170 device not ready");
            k_sleep(K_SECONDS(1));
            continue;
        }

        /* TMAG5170 SPI is pipelined: response to command N arrives during command N+1.
         * Fetch each channel TWICE — first call flushes the pipeline, second gets
         * the actual current register value. Using SENSOR_CHAN_ALL would shift all
         * channels by one (angle stored in drv_data->x, x in drv_data->y, etc.).
         */
        sensor_sample_fetch_chan(tmag, SENSOR_CHAN_ROTATION);   /* pipeline flush */
        int ret = sensor_sample_fetch_chan(tmag, SENSOR_CHAN_ROTATION);
        if (ret < 0) {
            LOG_ERR("sample_fetch ROTATION failed: %d", ret);
            k_sleep(K_SECONDS(1));
            continue;
        }

        sensor_channel_get(tmag, SENSOR_CHAN_ROTATION, &angle);

        LOG_INF("angle=%d.%04d deg", angle.val1, angle.val2 / 100);

        k_sleep(K_SECONDS(1));
    }

    return 0;
}
