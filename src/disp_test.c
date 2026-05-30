/*
 * SSD1309 display communication test.
 *
 * Self-contained – no dependencies on ui_display / BLE / haptic.
 * Cycles through text messages on the display so it's immediately obvious
 * whether the MCU is talking to the SSD1309 over SPI.
 *
 * Build:
 *   west build -b nrf5340dk/nrf5340/cpuapp -d build_disp --pristine -- \
 *     "-DDTC_OVERLAY_FILE=boards/nrf5340dk_nrf5340_cpuapp_ns.overlay;boards/nrf5340dk_nrf5340_cpuapp_disp.overlay" \
 *     "-DCONF_FILE=prj_disp.conf"
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(disp_test, LOG_LEVEL_INF);

/* ── Power rail GPIOs ────────────────────────────────────────────────
 * pwr_ctrl  P0.30 – TPS62740 CTRL: enables SSD1309 logic VDD
 * mic2288_mos P0.28 – MIC2288 boost gate: enables 12.6 V OLED panel VCC
 *
 * Both must be HIGH before the SSD1309 driver initialises (POST_KERNEL 91).
 * Only compiled when the nodes exist (custom PCB overlay).              */
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
    int ret;

#if DT_NODE_EXISTS(DT_NODELABEL(pwr_ctrl))
    printk("[pwr_ctrl] SYS_INIT entering, gpio_is_ready=%d\n",
           gpio_is_ready_dt(&g_pwr_ctrl));
    if (gpio_is_ready_dt(&g_pwr_ctrl)) {
        ret = gpio_pin_configure_dt(&g_pwr_ctrl, GPIO_OUTPUT_ACTIVE);
        printk("[pwr_ctrl] P0.30 HIGH (TPS62740 logic VDD ON), ret=%d\n", ret);
    } else {
        printk("[pwr_ctrl] GPIO not ready!\n");
    }
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(mic2288_mos))
    printk("[mic2288] SYS_INIT entering, gpio_is_ready=%d\n",
           gpio_is_ready_dt(&g_mic2288_mos));
    if (gpio_is_ready_dt(&g_mic2288_mos)) {
        ret = gpio_pin_configure_dt(&g_mic2288_mos, GPIO_OUTPUT_ACTIVE);
        printk("[mic2288] P0.28 HIGH (MIC2288 boost ON → 12.6V OLED VCC), ret=%d\n", ret);
    } else {
        printk("[mic2288] GPIO not ready!\n");
    }
#endif

    /* Give both regulators time to stabilise before the SSD1309 driver
     * runs at POST_KERNEL 90.  ready-time-ms=200 in DT adds further margin. */
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

/* ── SPI connectivity diagnostics ───────────────────────────────────────
 *
 * Run once at startup. Three tests, each with a log message explaining
 * the expected visual result.
 *
 * DIAG-1 (REVERSE DISPLAY command only, no data write):
 *   Sends 0xA7 via D/C=LOW (command mode).
 *   Power-on GDDRAM is all-zero; inverted mode → all pixels ON.
 *   If display stays BLACK: SPI command lane is dead (check D/C, CS, CLK,
 *   MOSI, RST pin connectivity and power).
 *
 * DIAG-2 (all-0xFF data write, normal display):
 *   Writes 1024 bytes of 0xFF via D/C=HIGH (data mode) → all pixels ON.
 *   If DIAG-1 was WHITE but DIAG-2 is BLACK: D/C polarity on data phase
 *   is wrong, or SPI MOSI/CLK signals don't reach the panel.
 *
 * DIAG-3 (all-0x00 data write):
 *   Confirms we can write zeros; display should be entirely BLACK.
 */
static void diagnostic_spi_connectivity(const struct device *disp)
{
    LOG_INF("--- DIAG-1: REVERSE DISPLAY (cmd only, no data) ---");
    LOG_INF("  Expected: entire display WHITE for ~4 s");
    LOG_INF("  If BLACK: SPI command path broken (D/C, CS, CLK, power)");
    fb_clear();
    fb_flush(disp);                                      /* ensure GDDRAM = 0 */
    display_set_pixel_format(disp, PIXEL_FORMAT_MONO10); /* sends 0xA7 */
    k_sleep(K_MSEC(4000));
    display_set_pixel_format(disp, PIXEL_FORMAT_MONO01); /* sends 0xA6 */
    k_sleep(K_MSEC(300));

    LOG_INF("--- DIAG-2: all-0xFF buffer write ---");
    LOG_INF("  Expected: entire display WHITE for ~4 s");
    LOG_INF("  If BLACK (DIAG-1 was white): data path / D/C polarity issue");
    memset(g_fb, 0xFF, BUF_LEN);
    fb_flush(disp);
    k_sleep(K_MSEC(4000));

    LOG_INF("--- DIAG-3: all-0x00 buffer write ---");
    LOG_INF("  Expected: entirely BLACK for ~2 s");
    fb_clear();
    fb_flush(disp);
    k_sleep(K_MSEC(2000));

    LOG_INF("--- Diagnostics done, entering normal loop ---");
}

/* ── Entry point ─────────────────────────────────────────────────────── */

int main(void)
{
    LOG_INF("=== SSD1309 display test ===");

#if DT_NODE_EXISTS(DT_NODELABEL(pwr_ctrl))
    /* Verify / re-assert pwr_ctrl. */
    if (gpio_is_ready_dt(&g_pwr_ctrl)) {
        int r = gpio_pin_configure_dt(&g_pwr_ctrl, GPIO_OUTPUT_ACTIVE);
        LOG_INF("pwr_ctrl  P0.30: configure=%d (TPS62740 logic VDD ON)", r);
    } else {
        LOG_ERR("pwr_ctrl GPIO not ready in main()!");
    }
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(mic2288_mos))
    /* Verify / re-assert MIC2288 boost enable. */
    if (gpio_is_ready_dt(&g_mic2288_mos)) {
        int r = gpio_pin_configure_dt(&g_mic2288_mos, GPIO_OUTPUT_ACTIVE);
        LOG_INF("mic2288   P0.28: configure=%d (boost ON → 12.6V OLED VCC)", r);
    } else {
        LOG_ERR("mic2288_mos GPIO not ready in main()!");
    }
#endif
    k_sleep(K_MSEC(50)); /* extra stabilisation margin */

    const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(disp)) {
        LOG_ERR("Display device not ready – check SPI pinctrl and overlay");
        return -1;
    }

    display_blanking_off(disp);
    LOG_INF("Display ready, running SPI diagnostics first...");
    diagnostic_spi_connectivity(disp);
    LOG_INF("Starting normal display loop");

    int frame = 0;
    while (1) {
        screen_hello(disp);
        LOG_INF("[%d] hello screen", frame++);
        k_sleep(K_MSEC(1500));

        screen_counter(disp, frame);
        LOG_INF("[%d] counter screen", frame++);
        k_sleep(K_MSEC(1500));

        screen_checkerboard(disp);
        LOG_INF("[%d] checkerboard", frame++);
        k_sleep(K_MSEC(1000));

        screen_stripes(disp);
        LOG_INF("[%d] stripes", frame++);
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
