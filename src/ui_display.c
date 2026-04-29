#include "ui_display.h"
#include "BLE_commands.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(ui_display, LOG_LEVEL_DBG);

#define DISP_W  128
#define DISP_H  64
#define BUF_LEN (DISP_W * DISP_H / 8)

static const struct device *g_disp;
static uint8_t g_fb[BUF_LEN];

/* ---- minimal 5x7 font (ASCII 0x20-0x7E) ---- */
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

/* Draw one character at pixel position (px, py) into g_fb.
 * MONO01: bit0 of byte = topmost pixel of the column stored in GRAM row order.
 * The SSD1306 driver uses page addressing: each byte covers 8 rows in one column.
 * Byte index = (row/8)*128 + col, bit = row%8. */
static void fb_putchar(int px, int py, char c)
{
    if (c < 0x20 || c > 0x7E) return;
    const uint8_t *glyph = font5x7[c - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                int x = px + col;
                int y = py + row;
                if (x < DISP_W && y < DISP_H) {
                    int byte_idx = (y / 8) * DISP_W + x;
                    g_fb[byte_idx] |= (1 << (y % 8));
                }
            }
        }
    }
}

static void fb_puts(int px, int py, const char *s)
{
    while (*s) {
        fb_putchar(px, py, *s++);
        px += 6; /* 5px glyph + 1px spacing */
    }
}

static void fb_flush(void)
{
    struct display_buffer_descriptor desc = {
        .buf_size = BUF_LEN,
        .width    = DISP_W,
        .height   = DISP_H,
        .pitch    = DISP_W,
    };
    display_write(g_disp, 0, 0, &desc, g_fb);
}

/* ------------------------------------------------------------------ */
/* Display state                                                        */
/* ------------------------------------------------------------------ */
#define MENU_MAX_ITEMS 16
#define MENU_ROWS      7     /* visible rows (each 9px tall: 7px font + 2px gap) */
#define ROW_H          9     /* pixels per row */
#define HEADER_H       10    /* pixels for header line + separator */
#define FOOTER_Y      55    /* y-coordinate of footer: back / confirm buttons */

static ui_disp_mode_t g_disp_mode = UI_DISP_STATUS;

/* Edit screen state */
static char   g_edit_name[16];
static char   g_edit_value_str[16];

/* Printer status snapshot (updated via ui_display_update_status) */
static struct ble_printer_state g_printer_state;  /* zero-initialised = no data */

/* Snapshot of menu data for re-render */
static char   g_menu_items[MENU_MAX_ITEMS][16];
static int    g_menu_count   = 0;
static int    g_menu_sel     = 0;
static char   g_menu_header[16];

/* -------------------------------------------------------------------
 * Dirty-flag rendering: haptic thread only sets flags, main loop
 * performs the actual SPI write via ui_display_process().
 * -------------------------------------------------------------------
 * g_render_pending : 1 = full re-render needed
 * g_flash_btn      : 0 = none, 1 = confirm, 2 = back (overlay for one frame)
 * ------------------------------------------------------------------- */
static atomic_t g_render_pending = ATOMIC_INIT(0);
static atomic_t g_flash_btn      = ATOMIC_INIT(0);

/* Restores normal rendering after flash overlay (scheduled 120 ms later) */
static struct k_work_delayable g_flash_restore_work;
static void flash_restore_fn(struct k_work *w)
{
    ARG_UNUSED(w);
    atomic_set(&g_flash_btn, 0);
    atomic_set(&g_render_pending, 1);
}

static void request_render(void)
{
    atomic_set(&g_render_pending, 1);
}

int ui_display_init(void)
{
    g_disp = DEVICE_DT_GET(DT_NODELABEL(ssd1309));
    if (!device_is_ready(g_disp)) {
        LOG_ERR("SSD1309 device not ready");
        return -ENODEV;
    }
    LOG_INF("SSD1309 ready");
    display_blanking_off(g_disp);
    k_work_init_delayable(&g_flash_restore_work, flash_restore_fn);
    return 0;
}

int ui_display_show_hello_remote(void)
{
    memset(g_fb, 0x00, BUF_LEN); /* clear to black */
    /* Center "Hello Remote" (12 chars * 6px = 72px wide) at y=28 */
    fb_puts((DISP_W - 12 * 6) / 2, 28, "Hello Remote");
    fb_flush();
    LOG_INF("Hello Remote written to display");
    return 0;
}

/* Draw a horizontal line across full width at pixel row y */
static void fb_hline(int y)
{
    if (y < 0 || y >= DISP_H) return;
    int byte_idx = (y / 8) * DISP_W;
    uint8_t bit  = (uint8_t)(1 << (y % 8));
    for (int x = 0; x < DISP_W; x++) {
        g_fb[byte_idx + x] |= bit;
    }
}

/* Invert (XOR) a rectangular region - use AFTER drawing text to get
 * black-text-on-white-background */
static void fb_invert_rect(int x, int y, int w, int h)
{
    for (int row = y; row < y + h && row < DISP_H; row++) {
        if (row < 0) continue;
        int byte_idx = (row / 8) * DISP_W;
        uint8_t bit  = (uint8_t)(1 << (row % 8));
        for (int col = x; col < x + w && col < DISP_W; col++) {
            if (col >= 0)
                g_fb[byte_idx + col] ^= bit;
        }
    }
}

/* Draw rectangle outline (border only, not filled) */
static void fb_rect_outline(int x, int y, int w, int h)
{
    /* top */
    for (int c = x; c < x + w && c < DISP_W; c++) {
        if (y >= 0 && y < DISP_H)
            g_fb[(y / 8) * DISP_W + c] |= (uint8_t)(1 << (y % 8));
    }
    /* bottom */
    int by = y + h - 1;
    for (int c = x; c < x + w && c < DISP_W; c++) {
        if (by >= 0 && by < DISP_H)
            g_fb[(by / 8) * DISP_W + c] |= (uint8_t)(1 << (by % 8));
    }
    /* left */
    for (int r = y; r < y + h && r < DISP_H; r++) {
        if (r >= 0 && x < DISP_W)
            g_fb[(r / 8) * DISP_W + x] |= (uint8_t)(1 << (r % 8));
    }
    /* right */
    int rx = x + w - 1;
    for (int r = y; r < y + h && r < DISP_H; r++) {
        if (r >= 0 && rx < DISP_W)
            g_fb[(r / 8) * DISP_W + rx] |= (uint8_t)(1 << (r % 8));
    }
}

static void render_edit(void)
{
    memset(g_fb, 0x00, BUF_LEN);

    /* Header: "EDIT: <name>" */
    char hdr[24];
    (void)snprintf(hdr, sizeof(hdr), "EDIT: %s", g_edit_name);
    int hx = (DISP_W - (int)strlen(hdr) * 6) / 2;
    if (hx < 0) hx = 0;
    fb_puts(hx, 1, hdr);
    fb_hline(9);

    /* Value box */
    fb_rect_outline(4, 19, DISP_W - 8, 20);

    /* Value string centred inside box */
    int vw = (int)strlen(g_edit_value_str) * 6;
    int vx = (DISP_W - vw) / 2;
    if (vx < 0) vx = 0;
    fb_puts(vx, 24, g_edit_value_str);

    /* Footer buttons: same layout as menu (normal state = outlined) */
    fb_rect_outline(0, FOOTER_Y, 63, DISP_H - FOOTER_Y);
    fb_puts(13, FOOTER_Y + 1, "< back");
    fb_rect_outline(65, FOOTER_Y, DISP_W - 65, DISP_H - FOOTER_Y);
    fb_puts(69, FOOTER_Y + 1, "confirm >");
    /* NOTE: no fb_flush() here — called by ui_display_process() */
}

/* Draw a 5×7 icon. cols[0..4] are column bytes with bit0=topmost pixel
 * (same format as font5x7). */
static void fb_icon_5x7(int px, int py, const uint8_t cols[5])
{
    for (int c = 0; c < 5; c++) {
        uint8_t bits = cols[c];
        for (int b = 0; b < 7; b++) {
            if (bits & (1u << b)) {
                int x = px + c;
                int y = py + b;
                if (x >= 0 && x < DISP_W && y >= 0 && y < DISP_H) {
                    g_fb[(y / 8) * DISP_W + x] |= (uint8_t)(1u << (y % 8));
                }
            }
        }
    }
}

/* 5×7 icon bitmaps (column-byte format, bit0=top, same as font5x7).
 * Visualised as rows 0-6 (top→bottom), left column first:
 *
 * icon_hotend — inverted pyramid (heater block → nozzle tip):
 *   ##  ##  #####  .###.  .###.  ..#..
 *
 * icon_bed — flat platform with two support legs:
 *   .....  #####  .....  #...#  #...#
 *
 * icon_fan — asterisk / propeller:
 *   #.#.#  .###.  #####  .###.  #.#.#
 *
 * icon_feed — clock face with hand (speedometer):
 *   .###.  #...#  #.##.  #....  .###.
 */
static const uint8_t icon_hotend[5] = { 0x03, 0x0F, 0x1F, 0x0F, 0x03 };
static const uint8_t icon_bed[5]    = { 0x34, 0x04, 0x04, 0x04, 0x34 };
static const uint8_t icon_fan[5]    = { 0x15, 0x0E, 0x1F, 0x0E, 0x15 };
static const uint8_t icon_feed[5]   = { 0x0E, 0x11, 0x15, 0x15, 0x02 };

static void render_status(void)
{
    memset(g_fb, 0x00, BUF_LEN);

    const struct ble_printer_state *s = &g_printer_state;
    char buf[22];

    /* ---- Row 0 (y=2): hotend icon | temp | fan icon | fan% ---- */
    fb_icon_5x7(2, 2, icon_hotend);
    if (s->valid) {
        (void)snprintf(buf, sizeof(buf), "%3.0f'", (double)s->temp_e);
    } else {
        (void)strncpy(buf, "---", sizeof(buf));
    }
    fb_puts(10, 2, buf);

    fb_icon_5x7(70, 2, icon_fan);
    if (s->valid) {
        (void)snprintf(buf, sizeof(buf), "%3.0f%%", (double)s->fan_pct);
    } else {
        (void)strncpy(buf, "--", sizeof(buf));
    }
    fb_puts(78, 2, buf);

    /* ---- Row 1 (y=13): bed icon | temp | feed icon | feed% ---- */
    fb_icon_5x7(2, 13, icon_bed);
    if (s->valid) {
        (void)snprintf(buf, sizeof(buf), "%3.0f'", (double)s->temp_b);
    } else {
        (void)strncpy(buf, "---", sizeof(buf));
    }
    fb_puts(10, 13, buf);

    fb_icon_5x7(70, 13, icon_feed);
    if (s->valid) {
        (void)snprintf(buf, sizeof(buf), "%3.0f%%", (double)s->feed_rate_pct);
    } else {
        (void)strncpy(buf, "--", sizeof(buf));
    }
    fb_puts(78, 13, buf);

    /* ---- Row 2 (y=25): progress bar — only when printing ---- */
    if (s->valid && s->printing) {
        fb_rect_outline(2, 25, 76, 11);
        float prog = s->progress_pct;
        int fill_w = (int)(74.0f * prog / 100.0f);
        if (fill_w > 74) fill_w = 74;
        for (int r = 26; r <= 34; r++) {
            int bi = (r / 8) * DISP_W;
            uint8_t bit = (uint8_t)(1 << (r % 8));
            for (int c = 3; c < 3 + fill_w; c++) {
                g_fb[bi + c] |= bit;
            }
        }
        (void)snprintf(buf, sizeof(buf), "%3.0f%%", (double)prog);
        int tw = (int)strlen(buf) * 6;
        int tx = 2 + (76 - tw) / 2;
        fb_puts(tx, 27, buf);
        if (fill_w > 0) {
            fb_invert_rect(3, 26, fill_w, 9);
        }

        /* Print time to the right of bar */
        uint32_t ptime = s->print_secs;
        uint32_t hh = ptime / 3600U;
        uint32_t mm = (ptime % 3600U) / 60U;
        (void)snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)hh, (unsigned)mm);
        fb_puts(84, 27, buf);
    }

    /* ---- Row 3 (y=39): status string ---- */
    const char *status_str;
    if (!s->valid) {
        status_str = "No printer";
    } else if (s->status_msg[0]) {
        status_str = s->status_msg;
    } else {
        status_str = "Ready";
    }
    fb_puts(2, 39, status_str);

    /* NOTE: no fb_flush() here — called by ui_display_process() */
}

static void render_menu(void)
{
    memset(g_fb, 0x00, BUF_LEN);

    /* Header */
    int hx = (DISP_W - (int)strlen(g_menu_header) * 6) / 2;
    fb_puts(hx < 0 ? 0 : hx, 1, g_menu_header);
    fb_hline(9);

    /* Scroll window: 5 rows above footer */
    int avail_h = FOOTER_Y - HEADER_H;   /* 45 px */
    int visible = avail_h / ROW_H;        /* = 5 rows */

    int scroll = g_menu_sel - visible / 2;
    if (scroll > g_menu_count - visible) scroll = g_menu_count - visible;
    if (scroll < 0) scroll = 0;

    for (int i = 0; i < visible && (scroll + i) < g_menu_count; i++) {
        int idx = scroll + i;
        int ry  = HEADER_H + i * ROW_H;
        bool selected = (idx == g_menu_sel);

        /* Arrow indicator */
        fb_puts(1, ry, selected ? ">" : " ");
        /* Item text */
        fb_puts(8, ry, g_menu_items[idx]);

        /* Outline around selected item during navigation */
        if (selected) {
            fb_rect_outline(0, ry - 1, DISP_W - 3, ROW_H);
        }
    }

    /* Scrollbar if needed */
    if (g_menu_count > visible) {
        int bar_h   = avail_h;
        int thumb_h = bar_h * visible / g_menu_count;
        if (thumb_h < 3) thumb_h = 3;
        int thumb_y = HEADER_H + (bar_h - thumb_h) * scroll / (g_menu_count - visible);
        /* track outline */
        fb_rect_outline(DISP_W - 2, HEADER_H, 2, bar_h);
        /* thumb filled (draw horizontal lines inside thumb) */
        for (int r = thumb_y; r < thumb_y + thumb_h && r < DISP_H; r++) {
            int bi = (r / 8) * DISP_W;
            uint8_t bit = (uint8_t)(1 << (r % 8));
            g_fb[bi + DISP_W - 2] |= bit;
            g_fb[bi + DISP_W - 1] |= bit;
        }
    }

    /* Footer: back (left) and confirm (right) navigation buttons */
    fb_rect_outline(0, FOOTER_Y, 63, DISP_H - FOOTER_Y);
    fb_puts(13, FOOTER_Y + 1, "< back");
    fb_rect_outline(65, FOOTER_Y, DISP_W - 65, DISP_H - FOOTER_Y);
    fb_puts(69, FOOTER_Y + 1, "confirm >");
    /* NOTE: no fb_flush() here — called by ui_display_process() */
}

void ui_display_set_mode(ui_disp_mode_t mode)
{
    g_disp_mode = mode;
    request_render();
}

void ui_display_update_edit(const char *param_name, const char *value_str)
{
    strncpy(g_edit_name, param_name, sizeof(g_edit_name) - 1);
    g_edit_name[sizeof(g_edit_name) - 1] = '\0';
    strncpy(g_edit_value_str, value_str, sizeof(g_edit_value_str) - 1);
    g_edit_value_str[sizeof(g_edit_value_str) - 1] = '\0';
    g_disp_mode = UI_DISP_EDIT;
    request_render();
}

/* Flash confirm button: set atomic overlay flag, re-render, restore after 120 ms.
 * Safe to call from any thread — no SPI, no sleep. */
void ui_display_flash_confirm(void)
{
    atomic_set(&g_flash_btn, 1);
    request_render();
    k_work_reschedule(&g_flash_restore_work, K_MSEC(120));
}

/* Flash back button — same pattern. */
void ui_display_flash_back(void)
{
    atomic_set(&g_flash_btn, 2);
    request_render();
    k_work_reschedule(&g_flash_restore_work, K_MSEC(120));
}

void ui_display_update_status(const struct ble_printer_state *state)
{
    g_printer_state = *state;
    if (g_disp_mode == UI_DISP_STATUS) {
        request_render();
    }
}

void ui_display_update_menu(menu_level_t level, int selected, int item_count,
                            const char * const *item_names, const char *header)
{
    ARG_UNUSED(level);

    g_menu_sel   = selected;
    g_menu_count = item_count < MENU_MAX_ITEMS ? item_count : MENU_MAX_ITEMS;

    strncpy(g_menu_header, header, sizeof(g_menu_header) - 1);
    g_menu_header[sizeof(g_menu_header) - 1] = '\0';

    for (int i = 0; i < g_menu_count; i++) {
        strncpy(g_menu_items[i], item_names[i], sizeof(g_menu_items[i]) - 1);
        g_menu_items[i][sizeof(g_menu_items[i]) - 1] = '\0';
    }

    g_disp_mode = UI_DISP_MENU;
    request_render();
}

void ui_display_process(void)
{
    /* Only render when something has changed — avoids SPI traffic on idle. */
    if (!atomic_cas(&g_render_pending, 1, 0)) {
        return;
    }

    /* Render content to framebuffer (no SPI yet) */
    switch (g_disp_mode) {
    case UI_DISP_STATUS: render_status(); break;
    case UI_DISP_EDIT:   render_edit();   break;
    default:             render_menu();   break;
    }

    /* Apply one-frame flash overlay if active */
    int flash = (int)atomic_get(&g_flash_btn);
    if (flash == 1) {
        fb_invert_rect(65, FOOTER_Y, DISP_W - 65, DISP_H - FOOTER_Y);
    } else if (flash == 2) {
        fb_invert_rect(0, FOOTER_Y, 63, DISP_H - FOOTER_Y);
    }

    /* Single SPI write — happens only in main thread, never in haptic thread */
    fb_flush();
}
