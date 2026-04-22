#include "ssd1309_display.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(display_ssd1309, LOG_LEVEL_INF);

#define DISP_SPI_NODE DT_NODELABEL(spi3)
#define DISP_DC_NODE  DT_ALIAS(disp_dc)
#define DISP_RST_NODE DT_ALIAS(disp_rst)
#define DISP_CS_NODE  DT_ALIAS(disp_cs)

#if !DT_NODE_EXISTS(DISP_SPI_NODE)
#error "DISP SPI node not found"
#endif

#if !DT_NODE_EXISTS(DISP_DC_NODE) || !DT_NODE_EXISTS(DISP_RST_NODE) || !DT_NODE_EXISTS(DISP_CS_NODE)
#error "Display GPIO aliases disp-dc/disp-rst/disp-cs are missing"
#endif

#define DISP_WIDTH   128
#define DISP_PAGES   8

static const struct device *g_spi_dev = DEVICE_DT_GET(DISP_SPI_NODE);
static const struct gpio_dt_spec g_dc = GPIO_DT_SPEC_GET(DISP_DC_NODE, gpios);
static const struct gpio_dt_spec g_rst = GPIO_DT_SPEC_GET(DISP_RST_NODE, gpios);
static const struct gpio_dt_spec g_cs = GPIO_DT_SPEC_GET(DISP_CS_NODE, gpios);

/*
 * Zephyr 4.x: spi_config.cs is an embedded struct spi_cs_control, not a pointer.
 * The gpio field is filled in at runtime inside ssd1309_display_init().
 */
static struct spi_config g_spi_cfg = {
    .frequency = 8000000U,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave     = 0,
    .cs = {
        .gpio  = { .port = NULL, .pin = 0, .dt_flags = 0 },
        .delay = 1U,
    },
};

static bool g_display_ready;

int ssd1309_display_write_cmd(uint8_t cmd)
{
    int ret = gpio_pin_set_dt(&g_dc, 0);
    if (ret < 0) {
        return ret;
    }

    const struct spi_buf tx_buf = {
        .buf = &cmd,
        .len = 1,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    return spi_write(g_spi_dev, &g_spi_cfg, &tx);
}

int ssd1309_display_write_data(const uint8_t *data, size_t len)
{
    int ret = gpio_pin_set_dt(&g_dc, 1);
    if (ret < 0) {
        return ret;
    }

    const struct spi_buf tx_buf = {
        .buf = (void *)data,
        .len = len,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    return spi_write(g_spi_dev, &g_spi_cfg, &tx);
}

int ssd1309_display_set_page_col(uint8_t page, uint8_t col)
{
    int ret = ssd1309_display_write_cmd((uint8_t)(0xB0U | (page & 0x0FU)));
    if (ret < 0) {
        return ret;
    }

    ret = ssd1309_display_write_cmd((uint8_t)(0x00U | (col & 0x0FU)));
    if (ret < 0) {
        return ret;
    }

    return ssd1309_display_write_cmd((uint8_t)(0x10U | ((col >> 4) & 0x0FU)));
}

int ssd1309_display_init(void)
{
    if (!device_is_ready(g_spi_dev)) {
        LOG_ERR("Display SPI device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&g_dc) || !gpio_is_ready_dt(&g_rst) || !gpio_is_ready_dt(&g_cs)) {
        LOG_ERR("Display GPIO not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&g_dc, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&g_rst, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&g_cs, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    g_spi_cfg.cs.gpio = g_cs;

    (void)gpio_pin_set_dt(&g_rst, 1);
    k_msleep(1);
    (void)gpio_pin_set_dt(&g_rst, 0);
    k_msleep(10);
    (void)gpio_pin_set_dt(&g_rst, 1);
    k_msleep(10);

    static const uint8_t init_seq[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x02,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0x7F,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0x2E,
        0xAF,
    };

    for (size_t i = 0; i < sizeof(init_seq); i++) {
        ret = ssd1309_display_write_cmd(init_seq[i]);
        if (ret < 0) {
            LOG_ERR("Display init failed at byte %u (ret=%d)", (unsigned int)i, ret);
            return ret;
        }
    }

    g_display_ready = true;
    LOG_INF("SSD1309 display init done");

    return ssd1309_display_clear();
}

int ssd1309_display_clear(void)
{
    if (!g_display_ready) {
        return -EACCES;
    }

    uint8_t line[DISP_WIDTH];
    (void)memset(line, 0x00, sizeof(line));

    for (uint8_t page = 0; page < DISP_PAGES; page++) {
        int ret = ssd1309_display_set_page_col(page, 0);
        if (ret < 0) {
            return ret;
        }

        ret = ssd1309_display_write_data(line, sizeof(line));
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}
