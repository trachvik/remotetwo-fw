/*
 * SSD1309 OLED display test – RemoteTwo custom PCB.
 *
 * Build:
 *   west build -b nrf5340dk/nrf5340/cpuapp -d build_disp --pristine -- \
 *     "-DDTC_OVERLAY_FILE=boards/remotetwo_nrf5340_cpuapp_ns.overlay" \
 *     "-DCONF_FILE=prj_disp.conf"
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(disp_test, LOG_LEVEL_INF);

#define TMAG_NODE DT_NODELABEL(tmag5170)

/* ── Power rails (POST_KERNEL 10, before display driver at 91) ──────────
 * pwr_ctrl    P0.30 – TPS62740 CTRL: SSD1309 logic VDD
 * mic2288_mos P0.28 – MIC2288 boost gate: 12.6 V OLED panel VCC       */
#if DT_NODE_EXISTS(DT_NODELABEL(pwr_ctrl))
static const struct gpio_dt_spec g_pwr_ctrl =
    GPIO_DT_SPEC_GET(DT_NODELABEL(pwr_ctrl), gpios);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(mic2288_mos))
static const struct gpio_dt_spec g_mic2288_mos =
    GPIO_DT_SPEC_GET(DT_NODELABEL(mic2288_mos), gpios);
#endif

static int power_rails_enable(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(pwr_ctrl))
    if (gpio_is_ready_dt(&g_pwr_ctrl)) {
        gpio_pin_configure_dt(&g_pwr_ctrl, GPIO_OUTPUT_ACTIVE);
    }
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(mic2288_mos))
    if (gpio_is_ready_dt(&g_mic2288_mos)) {
        gpio_pin_configure_dt(&g_mic2288_mos, GPIO_OUTPUT_ACTIVE);
    }
#endif
    k_sleep(K_MSEC(20));
    return 0;
}
SYS_INIT(power_rails_enable, POST_KERNEL, 10);

/* ── Display geometry ───────────────────────────────────────────────── */
#define DISP_W   128
#define DISP_H    64
/* SSD1306/1309 page addressing: each byte = 8 rows of one column.
 * Buffer layout: byte = (row/8)*128 + col,  bit = row%8             */
#define BUF_LEN  (DISP_W * DISP_H / 8)   /* 1024 bytes               */

static uint8_t g_fb[BUF_LEN];

/* ── Minimal 5×7 font (ASCII 0x20–0x7E) ────────────────────────────── */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* '\'' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */
    {0x10,0x08,0x08,0x10,0x08}, /* '~' */
};

/* ── Framebuffer helpers ─────────────────────────────────────────────── */

static void fb_clear(void)
{
    memset(g_fb, 0x00, BUF_LEN);
}

static void fb_set_pixel(int x, int y, int on)
{
    if (x < 0 || x >= DISP_W || y < 0 || y >= DISP_H) {
        return;
    }
    int idx = (y / 8) * DISP_W + x;
    int bit = y % 8;
    if (on) {
        g_fb[idx] |= (uint8_t)(1u << bit);
    } else {
        g_fb[idx] &= (uint8_t)~(1u << bit);
    }
}

static void fb_putchar(int px, int py, char c)
{
    if (c < 0x20 || c > 0x7E) {
        return;
    }
    const uint8_t *glyph = font5x7[(int)(c - 0x20)];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (uint8_t)(1u << row)) {
                fb_set_pixel(px + col, py + row, 1);
            }
        }
    }
}

static void fb_puts(int px, int py, const char *s)
{
    int x = px;
    while (*s) {
        fb_putchar(x, py, *s++);
        x += 6; /* 5px glyph + 1px spacing */
    }
}

static void fb_hline(int y)
{
    for (int x = 0; x < DISP_W; x++) {
        fb_set_pixel(x, y, 1);
    }
}

static void fb_flush(const struct device *disp)
{
    static const struct display_buffer_descriptor desc = {
        .buf_size = BUF_LEN,
        .width    = DISP_W,
        .height   = DISP_H,
        .pitch    = DISP_W,
    };
    int ret = display_write(disp, 0, 0, &desc, g_fb);
    if (ret) {
        LOG_ERR("display_write failed: %d", ret);
    }
}

/* ── Test screens ────────────────────────────────────────────────────── */

static void screen_hello(const struct device *disp)
{
    fb_clear();
    fb_puts(16, 10, "RemoteTwo");
    fb_hline(20);
    fb_puts(10, 28, "Display OK");
    fb_hline(38);
    fb_puts(22, 50, "SPI OK");
    fb_flush(disp);
}

static void screen_counter(const struct device *disp, int n)
{
    char buf[24];
    fb_clear();
    fb_puts(4, 4, "SSD1309 test");
    fb_hline(14);
    snprintf(buf, sizeof(buf), "Frame: %d", n);
    fb_puts(4, 24, buf);
    fb_puts(4, 44, "MCU->SPI->OLED");
    fb_flush(disp);
}

static void screen_checkerboard(const struct device *disp)
{
    fb_clear();
    for (int y = 0; y < DISP_H; y++) {
        for (int x = 0; x < DISP_W; x++) {
            fb_set_pixel(x, y, ((x / 8 + y / 8) & 1));
        }
    }
    fb_flush(disp);
}

static void screen_stripes(const struct device *disp)
{
    fb_clear();
    for (int y = 0; y < DISP_H; y++) {
        if ((y / 8) & 1) {
            for (int x = 0; x < DISP_W; x++) {
                fb_set_pixel(x, y, 1);
            }
        }
    }
    fb_flush(disp);
}

/* Raw SPI access for register diagnostics.
 * TMAG5170 uses pipelined SPI: RX during transaction N = response to N-1.
 * Reading a register TWICE gives the actual current value on the 2nd read. */
#if DT_NODE_HAS_STATUS(TMAG_NODE, okay)
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

    /* 1st read: flushes pipeline (gets previous command response) */
    spi_transceive_dt(&tmag_spi, &tx_set, &rx_set);
    memset(rx, 0, sizeof(rx));
    /* 2nd read: gets current register value */
    spi_transceive_dt(&tmag_spi, &tx_set, &rx_set);
    return (uint16_t)((rx[1] << 8) | rx[2]);
}
#endif

/* ── Entry point ─────────────────────────────────────────────────────── */

int main(void)
{
    const struct device *disp = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
    bool display_enabled = false;
#if DT_NODE_HAS_STATUS(TMAG_NODE, okay)
    const struct device *const tmag = DEVICE_DT_GET(TMAG_NODE);
    const struct device *const spi_bus = DEVICE_DT_GET(DT_BUS(TMAG_NODE));
#endif

    if (disp != NULL) {
        if (device_is_ready(disp)) {
            display_blanking_off(disp);
            display_enabled = true;
            LOG_INF("Display ready");
        } else {
            LOG_INF("Display disabled for TMAG-only test");
        }
    } else {
        LOG_INF("No zephyr_display chosen node; running TMAG-only test");
    }

#if DT_NODE_HAS_STATUS(TMAG_NODE, okay)
    LOG_INF("TMAG5170 test start");
    LOG_INF("TMAG node status: %s", device_is_ready(tmag) ? "ready" : "not ready");
    LOG_INF("SPI bus for TMAG: %s (%s)", spi_bus->name,
            device_is_ready(spi_bus) ? "ready" : "not ready");

    if (device_is_ready(tmag)) {
        uint16_t dev_cfg = tmag_read_reg_raw(0x00);
        uint16_t sens_cfg = tmag_read_reg_raw(0x01);
        uint16_t test_cfg = tmag_read_reg_raw(0x0F);
        LOG_INF("POST-INIT regs: DEVICE_CONFIG=0x%04x (exp 0x0020)  "
                "SENSOR_CONFIG=0x%04x (exp 0x40C0)  TEST_CONFIG=0x%04x",
                dev_cfg, sens_cfg, test_cfg);
    }
#endif

    uint32_t phase_a_start_ms = k_uptime_get_32();
    bool phase_b_started = false;

    LOG_INF("A/B test start: PHASE-A=TMAG only (10 s), PHASE-B=%s", display_enabled ? "TMAG+display" : "TMAG only");

    int frame = 0;
    while (1) {
#if DT_NODE_HAS_STATUS(TMAG_NODE, okay)
        if (device_is_ready(tmag)) {
            struct sensor_value angle;

            sensor_sample_fetch_chan(tmag, SENSOR_CHAN_ROTATION); /* pipeline flush */
            int ret = sensor_sample_fetch_chan(tmag, SENSOR_CHAN_ROTATION);
            if (ret < 0) {
                LOG_ERR("sample_fetch ROTATION failed: %d", ret);
            } else {
                sensor_channel_get(tmag, SENSOR_CHAN_ROTATION, &angle);
                LOG_INF("angle=%d.%04d deg", angle.val1, angle.val2 / 100);
            }
        }
#endif

        if (!phase_b_started && display_enabled) {
            if ((k_uptime_get_32() - phase_a_start_ms) >= 10000U) {
                phase_b_started = true;
                LOG_INF("PHASE-B start: TMAG+display enabled");
            } else {
                /* PHASE-A: no display writes to isolate SPI traffic for TMAG only. */
                k_sleep(K_SECONDS(1));
                continue;
            }
        } else if (!display_enabled) {
            /* TMAG-only mode: keep looping sensor reads, no display traffic. */
            k_sleep(K_SECONDS(1));
            continue;
        }

        if (disp == NULL) {
            k_sleep(K_SECONDS(1));
            continue;
        }

        screen_hello(disp);
        k_sleep(K_MSEC(1500));

        screen_counter(disp, frame++);
        k_sleep(K_MSEC(1500));

        screen_checkerboard(disp);
        k_sleep(K_MSEC(1000));

        screen_stripes(disp);
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
