#include "remote_control.h"
#include "BLE_commands.h"
#include "haptic.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(remote_control, LOG_LEVEL_DBG);

typedef enum {
    UI_MODE_NAV,
    UI_MODE_EDIT,
} ui_mode_t;

typedef enum {
    EDIT_NONE,
    EDIT_POS_X,
    EDIT_POS_Y,
    EDIT_POS_Z,
    EDIT_POS_E,
    EDIT_TEMP_EXTRUDER,
    EDIT_TEMP_BED,
    EDIT_ZOFFSET,
} edit_target_t;

typedef struct {
    edit_target_t target;
    const char *name;
    const char *token;
    const char *fmt;
    float value_min;
    float value_max;
    float value_default;
    float step_smooth;
    float step_fine;
    float step_mid;
    float step_coarse;
} param_desc_t;

static const param_desc_t g_params[] = {
    { EDIT_POS_X,         "X",        "mv:x",   "%.1f",  -200.0f, 200.0f,   0.0f, 0.1f, 1.0f,  5.0f, 10.0f  },
    { EDIT_POS_Y,         "Y",        "mv:y",   "%.1f",  -200.0f, 200.0f,   0.0f, 0.1f, 1.0f,  5.0f, 10.0f  },
    { EDIT_POS_Z,         "Z",        "mv:z",   "%.1f",  -200.0f, 200.0f,   0.0f, 0.1f, 1.0f,  5.0f, 10.0f  },
    { EDIT_POS_E,         "E",        "mv:e",   "%.1f",  -200.0f, 200.0f,   0.0f, 0.1f, 1.0f,  5.0f, 10.0f  },
    { EDIT_TEMP_EXTRUDER, "extruder", "tp:e",   "%.1f",     0.0f, 300.0f, 200.0f, 0.1f, 1.0f,  5.0f, 10.0f  },
    { EDIT_TEMP_BED,      "bed",      "tp:b",   "%.1f",     0.0f, 120.0f,  60.0f, 0.1f, 1.0f,  5.0f, 10.0f  },
    { EDIT_ZOFFSET,       "z-offset", "offset", "%+.2f",   -5.0f,   5.0f,   0.0f, 0.01f, 0.01f, 0.05f, 0.1f  },
};

static menu_level_t      g_level      = MENU_LEVEL_ROOT;
static root_item_t       g_root_sel   = ROOT_STATUS_MENU;
static position_item_t   g_pos_sel    = POS_BACK;
static temperature_item_t g_temp_sel  = TEMP_BACK;
static tool_item_t       g_tool_sel   = TOOL_BACK;
static macros_item_t     g_macros_sel = MACROS_BACK;
static sheet_item_t      g_sheet_sel  = SHEET_BACK;
static ui_mode_t         g_ui_mode    = UI_MODE_NAV;
static edit_target_t     g_edit_target = EDIT_NONE;
static float             g_edit_value  = 0.0f;
static float             g_edit_base_value = 0.0f;
static int64_t           g_btn2_press_ms     = 0;
static bool              g_light_on = false;

#define BTN2_LONG_PRESS_MS 700

static int wrap_step_int(int value, int count, int dir)
{
    int next = value + dir;
    if (next >= count) {
        next = 0;
    }
    if (next < 0) {
        next = count - 1;
    }
    return next;
}

static float clampf(float value, float min_val, float max_val)
{
    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

static const param_desc_t *find_param_desc(edit_target_t target)
{
    for (size_t i = 0; i < ARRAY_SIZE(g_params); i++) {
        if (g_params[i].target == target) {
            return &g_params[i];
        }
    }
    return NULL;
}

static bool is_position_target(edit_target_t target)
{
    return (target == EDIT_POS_X || target == EDIT_POS_Y || target == EDIT_POS_Z || target == EDIT_POS_E);
}

static float current_edit_step(const param_desc_t *desc)
{
    int ns = haptic_get_num_steps();
    if (ns == 0) {
        return desc->step_smooth;
    }
    if (ns <= 8) {
        return desc->step_coarse;
    }
    if (ns <= 12) {
        return desc->step_mid;
    }
    return desc->step_fine;
}

/* ---- Name helpers ---- */

static const char *level_name(menu_level_t level)
{
    switch (level) {
    case MENU_LEVEL_ROOT:        return "ROOT";
    case MENU_LEVEL_POSITION:    return "POSITION";
    case MENU_LEVEL_TEMPERATURE: return "TEMPERATURE";
    case MENU_LEVEL_TOOL:        return "TOOL";
    case MENU_LEVEL_MACROS:      return "MACROS";
    case MENU_LEVEL_SHEET:       return "SHEET";
    default:                     return "?";
    }
}

static const char *root_item_name(root_item_t item)
{
    switch (item) {
    case ROOT_STATUS_MENU: return "status menu";
    case ROOT_POSITION:    return "position";
    case ROOT_TEMPERATURE: return "temperature";
    case ROOT_TOOL:        return "set print tool";
    case ROOT_ZOFFSET:     return "z-offset";
    case ROOT_MACROS:      return "macros";
    default:               return "?";
    }
}

static const char *position_item_name(position_item_t item)
{
    switch (item) {
    case POS_BACK: return "back";
    case POS_X:    return "X";
    case POS_Y:    return "Y";
    case POS_Z:    return "Z";
    case POS_E:    return "E";
    default:       return "?";
    }
}

static const char *temperature_item_name(temperature_item_t item)
{
    switch (item) {
    case TEMP_BACK:     return "back";
    case TEMP_EXTRUDER: return "extruder";
    case TEMP_BED:      return "bed";
    default:            return "?";
    }
}

static const char *tool_item_name(tool_item_t item)
{
    switch (item) {
    case TOOL_BACK: return "back";
    case TOOL_T0:   return "T0";
    case TOOL_T1:   return "T1";
    case TOOL_T2:   return "T2";
    case TOOL_T3:   return "T3";
    case TOOL_T4:   return "T4";
    case TOOL_T5:   return "T5";
    case TOOL_T6:   return "T6";
    case TOOL_T7:   return "T7";
    case TOOL_T8:   return "T8";
    default:        return "?";
    }
}

static const char *macros_item_name(macros_item_t item)
{
    switch (item) {
    case MACROS_BACK:  return "back";
    case MACROS_SHEET: return "change print sheet";
    case MACROS_LIGHT: return "printer light toggle";
    default:           return "?";
    }
}

static const char *sheet_item_name(sheet_item_t item)
{
    switch (item) {
    case SHEET_BACK:    return "back";
    case SHEET_CUSTOM0: return "custom0";
    case SHEET_CUSTOM1: return "custom1";
    case SHEET_CUSTOM2: return "custom2";
    default:            return "?";
    }
}

static const char *current_selection_name(void)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        const param_desc_t *desc = find_param_desc(g_edit_target);
        return (desc != NULL) ? desc->name : "edit";
    }
    switch (g_level) {
    case MENU_LEVEL_ROOT:        return root_item_name(g_root_sel);
    case MENU_LEVEL_POSITION:    return position_item_name(g_pos_sel);
    case MENU_LEVEL_TEMPERATURE: return temperature_item_name(g_temp_sel);
    case MENU_LEVEL_TOOL:        return tool_item_name(g_tool_sel);
    case MENU_LEVEL_MACROS:      return macros_item_name(g_macros_sel);
    case MENU_LEVEL_SHEET:       return sheet_item_name(g_sheet_sel);
    default:                     return "?";
    }
}

static void log_menu_state(const char *reason)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        LOG_INF("MENU %s | mode=EDIT | level=%s | selected=%s | value=%.2f",
                reason, level_name(g_level), current_selection_name(), (double)g_edit_value);
        return;
    }
    LOG_INF("MENU %s | mode=NAV | level=%s | selected=%s",
            reason, level_name(g_level), current_selection_name());
}

static void send_text_gcode(const char *code)
{
    ble_send_gcode(code);
    LOG_INF("GCODE tx: %s", code);
}

static void send_position_delta(char axis, float delta)
{
    char cmd[20];
    (void)snprintf(cmd, sizeof(cmd), "mv:%c:%.1f", axis, (double)delta);
    send_text_gcode(cmd);
}

static void enter_edit_mode(edit_target_t target)
{
    const param_desc_t *desc = find_param_desc(target);
    if (desc == NULL) {
        return;
    }

    g_edit_base_value = desc->value_default;
    g_edit_value = desc->value_default;

    if (is_position_target(target)) {
        struct ble_printer_state st;
        if (ble_printer_state_get(&st)) {
            switch (target) {
            case EDIT_POS_X:
                g_edit_base_value = st.pos_x;
                break;
            case EDIT_POS_Y:
                g_edit_base_value = st.pos_y;
                break;
            case EDIT_POS_Z:
                g_edit_base_value = st.pos_z;
                break;
            case EDIT_POS_E:
                g_edit_base_value = st.pos_e;
                break;
            default:
                break;
            }
            g_edit_value = g_edit_base_value;
        }
    }

    g_ui_mode = UI_MODE_EDIT;
    g_edit_target = target;
    log_menu_state("enter edit");
}

static void confirm_edit_and_send(void)
{
    const param_desc_t *desc = find_param_desc(g_edit_target);
    if (desc == NULL) {
        g_ui_mode = UI_MODE_NAV;
        g_edit_target = EDIT_NONE;
        return;
    }
    float tx_value = g_edit_value;
    if (is_position_target(g_edit_target)) {
        tx_value = g_edit_value - g_edit_base_value;
    }

    if (is_position_target(g_edit_target)) {
        /* Position is applied on each knob step; confirm only exits edit mode. */
        g_ui_mode = UI_MODE_NAV;
        g_edit_target = EDIT_NONE;
        log_menu_state("exit edit");
        return;
    }

    if ((strncmp(desc->token, "mv:", 3) == 0 || strcmp(desc->token, "offset") == 0) && !ble_printer_can_move()) {
        LOG_WRN("Move blocked: printer reports can_move=0");
        return;
    }

    char cmd[20];
    char val_str[12];
    (void)snprintf(val_str, sizeof(val_str), desc->fmt, (double)tx_value);
    (void)snprintf(cmd, sizeof(cmd), "%s:%s", desc->token, val_str);
    send_text_gcode(cmd);

    g_ui_mode = UI_MODE_NAV;
    g_edit_target = EDIT_NONE;
    log_menu_state("exit edit");
}

static void send_tool(tool_item_t item)
{
    char cmd[6];
    (void)snprintf(cmd, sizeof(cmd), "t:%d", (int)item - (int)TOOL_T0);
    send_text_gcode(cmd);
}

static void send_sheet(sheet_item_t item)
{
    char cmd[12];
    (void)snprintf(cmd, sizeof(cmd), "s:custom%d", (int)item - (int)SHEET_CUSTOM0);
    send_text_gcode(cmd);
}

static void toggle_light(void)
{
    g_light_on = !g_light_on;
    send_text_gcode(g_light_on ? "l:on" : "l:off");
}

/* ---- Navigation ---- */

static void on_knob_step(int dir)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        const param_desc_t *desc = find_param_desc(g_edit_target);
        if (desc == NULL) {
            return;
        }
        float prev_value = g_edit_value;
        float step = current_edit_step(desc);
        g_edit_value = clampf(g_edit_value + ((float)dir * step),
                              desc->value_min, desc->value_max);
        g_edit_value = roundf(g_edit_value * 100.0f) / 100.0f;

        if (is_position_target(g_edit_target)) {
            if (!ble_printer_can_move()) {
                LOG_WRN("Move blocked: printer reports can_move=0");
                g_edit_value = prev_value;
                return;
            }

            float delta = g_edit_value - prev_value;
            if (fabsf(delta) >= 0.0001f) {
                send_position_delta(desc->token[3], delta);
                g_edit_base_value += delta;
            }
        }

        LOG_INF("Edit dir=%d step=%.3f val=%.2f", dir, (double)step, (double)g_edit_value);
        log_menu_state("edit");
        return;
    }

    switch (g_level) {
    case MENU_LEVEL_ROOT:
        g_root_sel = (root_item_t)wrap_step_int((int)g_root_sel, ROOT_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_POSITION:
        g_pos_sel = (position_item_t)wrap_step_int((int)g_pos_sel, POS_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_TEMPERATURE:
        g_temp_sel = (temperature_item_t)wrap_step_int((int)g_temp_sel, TEMP_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_TOOL:
        g_tool_sel = (tool_item_t)wrap_step_int((int)g_tool_sel, TOOL_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_MACROS:
        g_macros_sel = (macros_item_t)wrap_step_int((int)g_macros_sel, MACROS_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_SHEET:
        g_sheet_sel = (sheet_item_t)wrap_step_int((int)g_sheet_sel, SHEET_ITEM_COUNT, dir);
        break;
    }

    LOG_INF("Nav dir=%d", dir);
    log_menu_state("navigate");
}

static void on_btn2_confirm(void)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        confirm_edit_and_send();
        return;
    }

    if (g_level == MENU_LEVEL_ROOT) {
        switch (g_root_sel) {
        case ROOT_STATUS_MENU:
            log_menu_state("confirm status");
            return;
        case ROOT_POSITION:
            g_level = MENU_LEVEL_POSITION;
            g_pos_sel = POS_BACK;
            log_menu_state("enter submenu");
            return;
        case ROOT_TEMPERATURE:
            g_level = MENU_LEVEL_TEMPERATURE;
            g_temp_sel = TEMP_BACK;
            log_menu_state("enter submenu");
            return;
        case ROOT_TOOL:
            g_level = MENU_LEVEL_TOOL;
            g_tool_sel = TOOL_BACK;
            log_menu_state("enter submenu");
            return;
        case ROOT_ZOFFSET:
            enter_edit_mode(EDIT_ZOFFSET);
            return;
        case ROOT_MACROS:
            g_level = MENU_LEVEL_MACROS;
            g_macros_sel = MACROS_BACK;
            log_menu_state("enter submenu");
            return;
        default:
            return;
        }
    }

    if (g_level == MENU_LEVEL_POSITION) {
        switch (g_pos_sel) {
        case POS_BACK:
            g_level = MENU_LEVEL_ROOT;
            g_root_sel = ROOT_POSITION;
            log_menu_state("back");
            return;
        case POS_X: enter_edit_mode(EDIT_POS_X); return;
        case POS_Y: enter_edit_mode(EDIT_POS_Y); return;
        case POS_Z: enter_edit_mode(EDIT_POS_Z); return;
        case POS_E: enter_edit_mode(EDIT_POS_E); return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_TEMPERATURE) {
        switch (g_temp_sel) {
        case TEMP_BACK:
            g_level = MENU_LEVEL_ROOT;
            g_root_sel = ROOT_TEMPERATURE;
            log_menu_state("back");
            return;
        case TEMP_EXTRUDER: enter_edit_mode(EDIT_TEMP_EXTRUDER); return;
        case TEMP_BED:      enter_edit_mode(EDIT_TEMP_BED);      return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_TOOL) {
        if (g_tool_sel == TOOL_BACK) {
            g_level = MENU_LEVEL_ROOT;
            g_root_sel = ROOT_TOOL;
            log_menu_state("back");
            return;
        }
        send_tool(g_tool_sel);
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_TOOL;
        log_menu_state("tool selected");
        return;
    }

    if (g_level == MENU_LEVEL_MACROS) {
        switch (g_macros_sel) {
        case MACROS_BACK:
            g_level = MENU_LEVEL_ROOT;
            g_root_sel = ROOT_MACROS;
            log_menu_state("back");
            return;
        case MACROS_SHEET:
            g_level = MENU_LEVEL_SHEET;
            g_sheet_sel = SHEET_BACK;
            log_menu_state("enter submenu");
            return;
        case MACROS_LIGHT:
            toggle_light();
            log_menu_state("light toggled");
            return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_SHEET) {
        if (g_sheet_sel == SHEET_BACK) {
            g_level = MENU_LEVEL_MACROS;
            g_macros_sel = MACROS_SHEET;
            log_menu_state("back");
            return;
        }
        send_sheet(g_sheet_sel);
        g_level = MENU_LEVEL_MACROS;
        g_macros_sel = MACROS_SHEET;
        log_menu_state("sheet selected");
        return;
    }
}

static void on_btn2_long_press(void)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        g_ui_mode = UI_MODE_NAV;
        g_edit_target = EDIT_NONE;
        log_menu_state("cancel edit");
        return;
    }

    if (g_level != MENU_LEVEL_ROOT) {
        g_level = MENU_LEVEL_ROOT;
        log_menu_state("back root");
    }
}

/* btn2 (SW1) P0.24 = INPUT_KEY_ENTER  → confirm / long-press = back to root
 * Navigation: haptic knob rotation via on_knob_step callback                */
static void input_cb(struct input_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);

    if (evt->code != INPUT_KEY_ENTER) {
        return;
    }
    if (evt->value == 1) {
        g_btn2_press_ms = k_uptime_get();
        return;
    }
    if (evt->value == 0) {
        int64_t dt = k_uptime_get() - g_btn2_press_ms;
        if (dt >= BTN2_LONG_PRESS_MS) {
            on_btn2_long_press();
        } else {
            on_btn2_confirm();
        }
    }
}

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

void remote_control_init(void)
{
    int err = ble_commands_init();
    if (err != 0) {
        LOG_ERR("BLE init failed (%d)", err);
    }

    g_level       = MENU_LEVEL_ROOT;
    g_root_sel    = ROOT_STATUS_MENU;
    g_pos_sel     = POS_BACK;
    g_temp_sel    = TEMP_BACK;
    g_tool_sel    = TOOL_BACK;
    g_macros_sel  = MACROS_BACK;
    g_sheet_sel   = SHEET_BACK;
    g_ui_mode     = UI_MODE_NAV;
    g_edit_target = EDIT_NONE;
    g_light_on    = false;
    haptic_set_step_callback(on_knob_step);
    LOG_INF("Remote menu init (btn2=confirm, long=back, knob=navigate)");
    log_menu_state("init");
}

