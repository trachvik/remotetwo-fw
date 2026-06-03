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
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "drivers/bldc_driver_3pwm.h"

LOG_MODULE_REGISTER(disp_test, LOG_LEVEL_INF);

/* ── Motor parameters (3PWM open-loop) ─────────────────────────────────────
 * Source: DRV_test branch — prj_drv.conf + nrf5340dk_nrf5340_cpuapp_drv.overlay */
#define DRV_PI            3.14159265358979323846f
#define DRV_2PI           (2.0f * DRV_PI)
#define POLE_PAIRS        7
#define VOLTAGE_SUPPLY    3.75f    /* V, actual VM on PCB */
#define OPEN_LOOP_VOLTAGE 1.5f     /* V, same setpoint as DRV_test */
#define ALIGN_ITERS       1000U    /* 1000 × 1000 µs = 1 s alignment */
#define ALIGN_VOLTAGE     1.5f     /* V, same setpoint as DRV_test */
#define TARGET_VEL_MECH   25.0f    /* rad/s mechanical, same as DRV_test */
#define LOOP_PERIOD_US    500U     /* 2 kHz control loop, same as DRV_test */
#define CONTROL_DT        (LOOP_PERIOD_US * 1.0e-6f)
#define LOG_EVERY_N       2000U    /* ≈ 1 log line/second */

static bldc_driver_3pwm_t g_drv;
static float g_shaft_angle = 0.0f;
static bool  g_drv_ready   = false;
static bool  g_motor_run   = false;

static inline float drv_norm_angle(float a);
static void drv_set_phase_voltage(bldc_driver_3pwm_t *drv, float uq, float angle_el);

#define MOTOR_THREAD_STACK_SIZE 1024
#define MOTOR_THREAD_PRIORITY   K_PRIO_PREEMPT(0)
K_THREAD_STACK_DEFINE(g_motor_stack, MOTOR_THREAD_STACK_SIZE);
static struct k_thread g_motor_thread;

static void motor_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    const float vel_elec = TARGET_VEL_MECH * (float)POLE_PAIRS;
    uint32_t iter = 0U;

    while (g_motor_run) {
        g_shaft_angle = drv_norm_angle(g_shaft_angle + TARGET_VEL_MECH * CONTROL_DT);
        const float angle_el = drv_norm_angle(g_shaft_angle * (float)POLE_PAIRS);
        drv_set_phase_voltage(&g_drv, OPEN_LOOP_VOLTAGE, angle_el);

        if (++iter >= LOG_EVERY_N) {
            iter = 0U;
            LOG_INF("shaft=%.3f rad  elec=%.3f rad  w_el=%.1f rad/s  dc=[%.3f %.3f %.3f]",
                    (double)g_shaft_angle,
                    (double)angle_el,
                    (double)vel_elec,
                    (double)g_drv.dc_a,
                    (double)g_drv.dc_b,
                    (double)g_drv.dc_c);
        }

        k_busy_wait(LOOP_PERIOD_US);
    }
}

static inline float drv_norm_angle(float a)
{
    float r = fmodf(a, DRV_2PI);
    return (r < 0.0f) ? r + DRV_2PI : r;
}

/** Sinusoidal phase voltages — inverse Park + Clarke, centred at Vdc/2.
 *  Matches set_phase_voltage() in DRV_test.c. */
static void drv_set_phase_voltage(bldc_driver_3pwm_t *drv, float uq, float angle_el)
{
    const float mid  = drv->voltage_power_supply * 0.5f;
    const float u_al = -uq * sinf(angle_el);
    const float u_be =  uq * cosf(angle_el);
    bldc_driver_3pwm_set_pwm(drv,
        mid + u_al,
        mid + (-u_al * 0.5f + 0.8660254f * u_be),
        mid + (-u_al * 0.5f - 0.8660254f * u_be));
}

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

/* ── TMAG display screen ─────────────────────────────────────────────── */

static void screen_angle(const struct device *disp, const struct sensor_value *angle)
{
    char buf[24];
    int marker_x;
    int angle_mdeg;

    fb_clear();
    fb_puts(18, 6, "RemoteTwo");
    fb_hline(18);
    fb_puts(4, 22, "TMAG angle:");

    snprintf(buf, sizeof(buf), "%d.%02d deg", angle->val1, angle->val2 / 10000);
    fb_puts(4, 32, buf);

    /* Motor duty cycle row */
    if (g_drv_ready) {
        snprintf(buf, sizeof(buf), "M:%2d %2d %2d%%",
                 (int)(g_drv.dc_a * 100.0f),
                 (int)(g_drv.dc_b * 100.0f),
                 (int)(g_drv.dc_c * 100.0f));
    } else {
        snprintf(buf, sizeof(buf), "Motor: offline");
    }
    fb_puts(4, 43, buf);

    angle_mdeg = (angle->val1 * 1000) + (angle->val2 / 1000);
    if (angle_mdeg < 0) {
        angle_mdeg = 0;
    }
    if (angle_mdeg > 360000) {
        angle_mdeg = 360000;
    }

    marker_x = 4 + (angle_mdeg * 119) / 360000;
    for (int y = 56; y < 63; y++) {
        fb_set_pixel(marker_x, y, 1);
    }
    for (int x = 4; x < 124; x++) {
        fb_set_pixel(x, 54, 1);
    }

    fb_flush(disp);
}

/* ── Entry point ─────────────────────────────────────────────────────── */

int main(void)
{
    const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
#if DT_NODE_HAS_STATUS(TMAG_NODE, okay)
    const struct device *const tmag = DEVICE_DT_GET(TMAG_NODE);
#endif

    if (!device_is_ready(disp)) {
        LOG_ERR("Display not ready");
        return -1;
    }

    display_blanking_off(disp);

#if DT_NODE_HAS_STATUS(TMAG_NODE, okay)
    if (!device_is_ready(tmag)) {
        LOG_ERR("TMAG not ready");
        return -1;
    }
#endif

    LOG_INF("Display + TMAG ready");

    /* ── DRV8311H motor driver init ─────────────────────────────────────── */
    bldc_driver_3pwm_init_struct(&g_drv);
    g_drv.voltage_power_supply = VOLTAGE_SUPPLY;
    g_drv.voltage_limit        = VOLTAGE_SUPPLY;
    g_drv.pwm_frequency        = 25000;

    if (bldc_driver_3pwm_init_hw(&g_drv) != DRIVER_INIT_OK) {
        LOG_ERR("bldc_driver_3pwm_init_hw failed — DRV8311H offline");
    } else {
        bldc_driver_3pwm_enable(&g_drv);
        g_drv_ready = true;
        LOG_INF("DRV8311H ready: nSLEEP=HIGH, INL=HIGH");

        /* ── Rotor alignment: hold angle_el=0 for 1 s ──────────────────
         * Pulls the rotor to a known electrical zero before open-loop spin.
         * Uses k_busy_wait (interrupts stay enabled — SPI still works). */
        LOG_INF("Aligning rotor (1 s)...");
        for (uint32_t i = 0; i < ALIGN_ITERS; ++i) {
            drv_set_phase_voltage(&g_drv, ALIGN_VOLTAGE, 0.0f);
            k_busy_wait(1000U);
        }

        g_motor_run = true;
        (void)k_thread_create(&g_motor_thread, g_motor_stack,
                              K_THREAD_STACK_SIZEOF(g_motor_stack),
                              motor_thread_fn,
                              NULL, NULL, NULL,
                              MOTOR_THREAD_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&g_motor_thread, "motor_ol");
        LOG_INF("Alignment done, starting open-loop spin");
    }

    LOG_INF("Combined test: TMAG5170 + SSD1309 + DRV8311H");

    while (1) {
        /* Display and TMAG are serviced every 100 ms in the main thread. */
#if DT_NODE_HAS_STATUS(TMAG_NODE, okay)
        struct sensor_value angle;

        int ret = sensor_sample_fetch_chan(tmag, SENSOR_CHAN_ROTATION);
        if (ret < 0) {
            LOG_ERR("TMAG sample fetch failed: %d", ret);
        } else {
            ret = sensor_channel_get(tmag, SENSOR_CHAN_ROTATION, &angle);
            if (ret < 0) {
                LOG_ERR("TMAG channel get failed: %d", ret);
            } else {
                screen_angle(disp, &angle);
            }
        }
#else
        fb_clear();
        fb_puts(16, 24, "TMAG missing");
        fb_puts(g_drv_ready ? 28 : 16, 38,
                g_drv_ready ? "DRV: OK" : "DRV: offline");
        fb_flush(disp);
#endif

        k_sleep(K_MSEC(100));
    }

    return 0;
}
