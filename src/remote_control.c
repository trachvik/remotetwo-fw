#include "remote_control.h"
#include "BLE_commands.h"
#include "haptic.h"
#include "ui_display.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(remote_control, LOG_LEVEL_DBG);

typedef enum {
    UI_MODE_NAV,
    UI_MODE_EDIT,
} ui_mode_t;

typedef enum {
    HAPTIC_MODE_MENU_HIDDEN,
    HAPTIC_MODE_MENU_VISIBLE,
} haptic_mode_menu_state_t;

typedef enum {
    HAPTIC_MODE_SMOOTH,
    HAPTIC_MODE_STEP_16,
    HAPTIC_MODE_STEP_12,
    HAPTIC_MODE_STEP_8,
    HAPTIC_MODE_ITEM_COUNT,
} haptic_mode_item_t;

typedef enum {
    EDIT_NONE,
    EDIT_POS_X,
    EDIT_POS_Y,
    EDIT_POS_Z,
    EDIT_POS_E,
    EDIT_TEMP_EXTRUDER,
    EDIT_TEMP_BED,
    EDIT_ZOFFSET,
    EDIT_FAN,
    EDIT_FLOW,
    EDIT_SPEED,
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
    { EDIT_FAN,           "fan",      "fan",    "%.0f",    0.0f, 100.0f,   0.0f, 1.0f, 5.0f, 10.0f, 25.0f  },
    { EDIT_FLOW,          "flow",     "flow",   "%.0f",   10.0f, 300.0f, 100.0f, 1.0f, 5.0f, 10.0f, 25.0f  },
    { EDIT_SPEED,         "speed",    "speed",  "%.0f",   10.0f, 300.0f, 100.0f, 1.0f, 5.0f, 10.0f, 25.0f  },
};

static menu_level_t      g_level         = MENU_LEVEL_ROOT;
static root_item_t       g_root_sel      = ROOT_STATUS_MENU;
static control_item_t    g_ctrl_sel      = CTRL_HOME_ALL;
static control_axis_item_t g_ctrl_axis_sel = CTRL_AXIS_HOME;
static int               g_ctrl_axis     = 0; /* 0=X, 1=Y, 2=Z */
static temperature_item_t g_temp_sel     = TEMP_EXTRUDER;
static filament_item_t   g_filament_sel  = FILAMENT_PREHEAT_PLA;
static calibration_item_t g_calib_sel    = CALIB_Z;
static mmu_item_t        g_mmu_sel       = MMU_HOME;
static mmu_tool_item_t   g_mmu_tool_sel  = MMU_T0;
static macros_item_t     g_macros_sel    = MACROS_SHEET;
static sheet_item_t      g_sheet_sel     = SHEET_CUSTOM0;
static printing_item_t   g_printing_sel  = PRINTING_PAUSE;
static ui_mode_t         g_ui_mode    = UI_MODE_NAV;
static edit_target_t     g_edit_target = EDIT_NONE;
static float             g_edit_value  = 0.0f;
static float             g_edit_base_value = 0.0f;
static bool              g_light_on = false;
static int               g_smooth_nav_accum = 0;
static haptic_mode_menu_state_t g_haptic_mode_menu = HAPTIC_MODE_MENU_HIDDEN;
static haptic_mode_item_t g_haptic_mode_sel = HAPTIC_MODE_SMOOTH;
static struct k_work_delayable g_click_eval_work;
static int g_pending_click_dir = 0;
/* UI updates are prepared by callbacks but pushed to display only from
 * remote_control_tick() in the main loop to avoid cross-thread UI races. */
static atomic_t g_menu_refresh_pending = ATOMIC_INIT(0);
static atomic_t g_status_mode_pending  = ATOMIC_INIT(0);

#define DOUBLE_CLICK_WINDOW_MS  350
#define SMOOTH_MENU_STEP_DIV    3
#define DISPLAY_IDLE_TIMEOUT_MS 20000

/* --- 20-second inactivity timer: switch back to status screen --- */
static struct k_work_delayable g_idle_disp_work;

/* True while the status screen is the active display. Cleared on first menu
 * navigation so that a knob turn from the status screen always resets to root. */
static bool g_is_status_mode = true;

static inline void request_menu_refresh(void)
{
    g_is_status_mode = false;
    atomic_set(&g_menu_refresh_pending, 1);
}

static inline void request_status_mode(void)
{
    g_is_status_mode = true;
    atomic_set(&g_status_mode_pending, 1);
    /* Prevent a stale pending menu refresh from immediately switching back. */
    atomic_set(&g_menu_refresh_pending, 0);
}

static void idle_disp_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    request_status_mode();
}

static void disp_reset_idle_timer(void)
{
    k_work_reschedule(&g_idle_disp_work, K_MSEC(DISPLAY_IDLE_TIMEOUT_MS));
}

/* Forward declaration (defined later in this file) */
static const param_desc_t *find_param_desc(edit_target_t target);

/* --- Build and push current menu to display --- */
static void display_push_menu(void)
{
    /* Edit mode: show value edit screen instead of menu */
    if (g_ui_mode == UI_MODE_EDIT) {
        const param_desc_t *desc = find_param_desc(g_edit_target);
        if (desc != NULL) {
            char val_str[16];
            (void)snprintf(val_str, sizeof(val_str), desc->fmt, (double)g_edit_value);
            ui_display_update_edit(desc->name, val_str);
            disp_reset_idle_timer();
        }
        return;
    }

    const char *names[16];
    int count = 0;
    int sel   = 0;
    const char *header = "";

    if (g_haptic_mode_menu == HAPTIC_MODE_MENU_VISIBLE) {
        header = "Haptic Mode";
        static const char *hm[] = {"Smooth","Step 16","Step 12","Step 8"};
        count = HAPTIC_MODE_ITEM_COUNT;
        for (int i = 0; i < count; i++) names[i] = hm[i];
        sel = (int)g_haptic_mode_sel;
        ui_display_update_menu(g_level, sel, count, names, header);
        disp_reset_idle_timer();
        return;
    }

    switch (g_level) {
    case MENU_LEVEL_ROOT:
        header = "Menu";
        names[0] = "Control";
        names[1] = "Temperature";
        names[2] = "Filament";
        names[3] = "SD Card";
        names[4] = "Calibration";
        names[5] = "MMU";
        names[6] = "Z-offset";
        names[7] = "Macros";
        names[8] = "Printing";
        count = ROOT_ITEM_COUNT - 1; /* skip ROOT_STATUS_MENU */
        sel = (int)g_root_sel - 1;
        if (sel < 0) sel = 0;
        break;
    case MENU_LEVEL_CONTROL:
        header = "Control";
        names[0] = "Home All";
        names[1] = "X"; names[2] = "Y"; names[3] = "Z";
        names[4] = "E";
        names[5] = "Fan Speed";
        names[6] = "Motors Off";
        count = CTRL_ITEM_COUNT;
        sel   = (int)g_ctrl_sel;
        break;
    case MENU_LEVEL_CONTROL_X:
    case MENU_LEVEL_CONTROL_Y:
    case MENU_LEVEL_CONTROL_Z:
        header = (g_level == MENU_LEVEL_CONTROL_X) ? "X" :
                 (g_level == MENU_LEVEL_CONTROL_Y) ? "Y" : "Z";
        names[0] = "Home"; names[1] = "Move";
        count = CTRL_AXIS_ITEM_COUNT;
        sel   = (int)g_ctrl_axis_sel;
        break;
    case MENU_LEVEL_TEMPERATURE:
        header = "Temperature";
        names[0] = "Extruder"; names[1] = "Bed"; names[2] = "Cool Down";
        count = TEMP_ITEM_COUNT;
        sel   = (int)g_temp_sel;
        break;
    case MENU_LEVEL_FILAMENT:
        header = "Filament";
        names[0] = "Preheat PLA"; names[1] = "Preheat PETG";
        names[2] = "Load"; names[3] = "Unload";
        count = FILAMENT_ITEM_COUNT;
        sel   = (int)g_filament_sel;
        break;
    case MENU_LEVEL_SD_CARD:
        header = "SD Card";
        names[0] = "(files)";
        count = 1;
        sel   = 0;
        break;
    case MENU_LEVEL_CALIBRATION:
        header = "Calibration";
        names[0] = "Calibrate Z"; names[1] = "Bed Mesh";
        names[2] = "First Layer"; names[3] = "Probe Cal.";
        count = CALIB_ITEM_COUNT;
        sel   = (int)g_calib_sel;
        break;
    case MENU_LEVEL_MMU:
        header = "MMU";
        names[0] = "Home"; names[1] = "Resume";
        names[2] = "Locate Sel."; names[3] = "Set Tool";
        count = MMU_ITEM_COUNT;
        sel   = (int)g_mmu_sel;
        break;
    case MENU_LEVEL_MMU_LOCATE:
    case MENU_LEVEL_MMU_SET_TOOL:
        header = (g_level == MENU_LEVEL_MMU_LOCATE) ? "Locate Sel." : "Set Tool";
        names[0]="T0"; names[1]="T1"; names[2]="T2"; names[3]="T3";
        names[4]="T4"; names[5]="T5"; names[6]="T6"; names[7]="T7"; names[8]="T8";
        count = MMU_TOOL_ITEM_COUNT;
        sel   = (int)g_mmu_tool_sel;
        break;
    case MENU_LEVEL_MACROS:
        header = "Macros";
        names[0] = "Change Sheet";
        names[1] = g_light_on ? "Light ON" : "Light OFF";
        names[2] = "Fake Position";
        count = MACROS_ITEM_COUNT;
        sel   = (int)g_macros_sel;
        break;
    case MENU_LEVEL_SHEET:
        header = "Sheet";
        names[0] = "Custom 0"; names[1] = "Custom 1"; names[2] = "Custom 2";
        count = SHEET_ITEM_COUNT;
        sel   = (int)g_sheet_sel;
        break;
    case MENU_LEVEL_PRINTING:
        header = "Printing";
        names[0] = "Pause"; names[1] = "Flow"; names[2] = "Speed";
        count = PRINTING_ITEM_COUNT;
        sel   = (int)g_printing_sel;
        break;
    default:
        return;
    }

    ui_display_update_menu(g_level, sel, count, names, header);
    disp_reset_idle_timer();
}

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

static int haptic_mode_item_to_steps(haptic_mode_item_t item)
{
    switch (item) {
    case HAPTIC_MODE_SMOOTH:  return 0;
    case HAPTIC_MODE_STEP_16: return 16;
    case HAPTIC_MODE_STEP_12: return 12;
    case HAPTIC_MODE_STEP_8:  return 8;
    default:                  return 0;
    }
}

static haptic_mode_item_t haptic_mode_steps_to_item(int steps)
{
    switch (steps) {
    case 16: return HAPTIC_MODE_STEP_16;
    case 12: return HAPTIC_MODE_STEP_12;
    case 8:  return HAPTIC_MODE_STEP_8;
    default: return HAPTIC_MODE_SMOOTH;
    }
}

static const char *haptic_mode_item_name(haptic_mode_item_t item)
{
    switch (item) {
    case HAPTIC_MODE_SMOOTH:  return "smooth";
    case HAPTIC_MODE_STEP_16: return "step-16";
    case HAPTIC_MODE_STEP_12: return "step-12";
    case HAPTIC_MODE_STEP_8:  return "step-8";
    default:                  return "?";
    }
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
    case MENU_LEVEL_ROOT:         return "ROOT";
    case MENU_LEVEL_CONTROL:      return "CONTROL";
    case MENU_LEVEL_CONTROL_X:    return "CONTROL_X";
    case MENU_LEVEL_CONTROL_Y:    return "CONTROL_Y";
    case MENU_LEVEL_CONTROL_Z:    return "CONTROL_Z";
    case MENU_LEVEL_TEMPERATURE:  return "TEMPERATURE";
    case MENU_LEVEL_FILAMENT:     return "FILAMENT";
    case MENU_LEVEL_SD_CARD:      return "SD_CARD";
    case MENU_LEVEL_CALIBRATION:  return "CALIBRATION";
    case MENU_LEVEL_MMU:          return "MMU";
    case MENU_LEVEL_MMU_LOCATE:   return "MMU_LOCATE";
    case MENU_LEVEL_MMU_SET_TOOL: return "MMU_SET_TOOL";
    case MENU_LEVEL_MACROS:       return "MACROS";
    case MENU_LEVEL_SHEET:        return "SHEET";
    case MENU_LEVEL_PRINTING:     return "PRINTING";
    default:                      return "?";
    }
}

static const char *root_item_name(root_item_t item)
{
    switch (item) {
    case ROOT_STATUS_MENU: return "status menu";
    case ROOT_CONTROL:     return "control";
    case ROOT_TEMPERATURE: return "temperature";
    case ROOT_FILAMENT:    return "filament";
    case ROOT_SD_CARD:     return "sd card";
    case ROOT_CALIBRATION: return "calibration";
    case ROOT_MMU:         return "mmu";
    case ROOT_ZOFFSET:     return "z-offset";
    case ROOT_MACROS:      return "macros";
    case ROOT_PRINTING:    return "printing";
    default:               return "?";
    }
}

static const char *macros_item_name(macros_item_t item)
{
    switch (item) {
    case MACROS_SHEET:         return "change print sheet";
    case MACROS_LIGHT:         return "printer light toggle";
    case MACROS_FAKE_POSITION: return "fake position";
    default:                   return "?";
    }
}

static const char *temperature_item_name(temperature_item_t item)
{
    switch (item) {
    case TEMP_EXTRUDER:  return "extruder";
    case TEMP_BED:       return "bed";
    case TEMP_COOL_DOWN: return "cool down";
    default:             return "?";
    }
}

static const char *sheet_item_name(sheet_item_t item)
{
    switch (item) {
    case SHEET_CUSTOM0: return "custom0";
    case SHEET_CUSTOM1: return "custom1";
    case SHEET_CUSTOM2: return "custom2";
    default:            return "?";
    }
}

static const char *current_selection_name(void)
{
    if (g_haptic_mode_menu == HAPTIC_MODE_MENU_VISIBLE) {
        return haptic_mode_item_name(g_haptic_mode_sel);
    }
    if (g_ui_mode == UI_MODE_EDIT) {
        const param_desc_t *desc = find_param_desc(g_edit_target);
        return (desc != NULL) ? desc->name : "edit";
    }
    switch (g_level) {
    case MENU_LEVEL_ROOT:         return root_item_name(g_root_sel);
    case MENU_LEVEL_CONTROL:
    case MENU_LEVEL_CONTROL_X:
    case MENU_LEVEL_CONTROL_Y:
    case MENU_LEVEL_CONTROL_Z:    return level_name(g_level);
    case MENU_LEVEL_TEMPERATURE:  return temperature_item_name(g_temp_sel);
    case MENU_LEVEL_MACROS:       return macros_item_name(g_macros_sel);
    case MENU_LEVEL_SHEET:        return sheet_item_name(g_sheet_sel);
    default:                      return level_name(g_level);
    }
}

static void log_menu_state(const char *reason)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        LOG_INF("MENU %s | mode=EDIT | level=%s | selected=%s | value=%.2f",
                reason, level_name(g_level), current_selection_name(), (double)g_edit_value);
    } else {
        LOG_INF("MENU %s | mode=NAV | level=%s | selected=%s",
                reason, level_name(g_level), current_selection_name());
    }
    request_menu_refresh();
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

    if (is_position_target(g_edit_target) || g_edit_target == EDIT_ZOFFSET) {
        /* Applied on each knob step; confirm only exits edit mode. */
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

static void send_simple_gcode(const char *code)
{
    send_text_gcode(code);
}

static void send_tool(mmu_tool_item_t item)
{
    char cmd[6];
    (void)snprintf(cmd, sizeof(cmd), "t:%d", (int)item - (int)MMU_T0);
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
    if (g_haptic_mode_menu == HAPTIC_MODE_MENU_VISIBLE) {
        g_haptic_mode_sel = (haptic_mode_item_t)wrap_step_int(
            (int)g_haptic_mode_sel, HAPTIC_MODE_ITEM_COUNT, dir);
        LOG_INF("Haptic mode select dir=%d", dir);
        log_menu_state("mode navigate");
        return;
    }

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
        } else if (g_edit_target == EDIT_ZOFFSET) {
            if (!ble_printer_can_move()) {
                LOG_WRN("Move blocked: printer reports can_move=0");
                g_edit_value = prev_value;
                return;
            }
            float delta = g_edit_value - prev_value;
            if (fabsf(delta) >= 0.0001f) {
                char cmd[20];
                char val_str[12];
                (void)snprintf(val_str, sizeof(val_str), desc->fmt, (double)delta);
                (void)snprintf(cmd, sizeof(cmd), "%s:%s", desc->token, val_str);
                send_text_gcode(cmd);
                g_edit_base_value += delta;
            }
        }

        LOG_INF("Edit dir=%d step=%.3f val=%.2f", dir, (double)step, (double)g_edit_value);
        log_menu_state("edit");
        return;
    }

    if (haptic_get_num_steps() == 0) {
        /* In smooth mode, make menu scrolling coarser while keeping edit mode fine. */
        if ((g_smooth_nav_accum > 0 && dir < 0) || (g_smooth_nav_accum < 0 && dir > 0)) {
            g_smooth_nav_accum = 0;
        }
        g_smooth_nav_accum += dir;
        if (g_smooth_nav_accum >= SMOOTH_MENU_STEP_DIV) {
            dir = 1;
            g_smooth_nav_accum = 0;
        } else if (g_smooth_nav_accum <= -SMOOTH_MENU_STEP_DIV) {
            dir = -1;
            g_smooth_nav_accum = 0;
        } else {
            return;
        }
    } else {
        g_smooth_nav_accum = 0;
    }

    /* When returning from the status screen, always start at the root menu
     * so the user never lands in an unexpected submenu. */
    if (g_is_status_mode) {
        g_level    = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_CONTROL;
    }

    switch (g_level) {
    case MENU_LEVEL_ROOT:
        {
            /* Navigate only among real menu items (skip ROOT_STATUS_MENU index 0) */
            int idx = (int)g_root_sel - 1;
            if (idx < 0) idx = 0;
            idx = wrap_step_int(idx, ROOT_ITEM_COUNT - 1, dir);
            g_root_sel = (root_item_t)(idx + 1);
        }
        break;
    case MENU_LEVEL_CONTROL:
        g_ctrl_sel = (control_item_t)wrap_step_int((int)g_ctrl_sel, CTRL_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_CONTROL_X:
    case MENU_LEVEL_CONTROL_Y:
    case MENU_LEVEL_CONTROL_Z:
        g_ctrl_axis_sel = (control_axis_item_t)wrap_step_int((int)g_ctrl_axis_sel, CTRL_AXIS_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_TEMPERATURE:
        g_temp_sel = (temperature_item_t)wrap_step_int((int)g_temp_sel, TEMP_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_FILAMENT:
        g_filament_sel = (filament_item_t)wrap_step_int((int)g_filament_sel, FILAMENT_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_SD_CARD:
        /* SD card file list — not yet implemented, no navigation */
        break;
    case MENU_LEVEL_CALIBRATION:
        g_calib_sel = (calibration_item_t)wrap_step_int((int)g_calib_sel, CALIB_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_MMU:
        g_mmu_sel = (mmu_item_t)wrap_step_int((int)g_mmu_sel, MMU_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_MMU_LOCATE:
    case MENU_LEVEL_MMU_SET_TOOL:
        g_mmu_tool_sel = (mmu_tool_item_t)wrap_step_int((int)g_mmu_tool_sel, MMU_TOOL_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_MACROS:
        g_macros_sel = (macros_item_t)wrap_step_int((int)g_macros_sel, MACROS_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_SHEET:
        g_sheet_sel = (sheet_item_t)wrap_step_int((int)g_sheet_sel, SHEET_ITEM_COUNT, dir);
        break;
    case MENU_LEVEL_PRINTING:
        g_printing_sel = (printing_item_t)wrap_step_int((int)g_printing_sel, PRINTING_ITEM_COUNT, dir);
        break;
    }

    LOG_INF("Nav dir=%d", dir);
    log_menu_state("navigate");
}

static void on_confirm_action(void)
{
    ui_display_flash_confirm();

    if (g_haptic_mode_menu == HAPTIC_MODE_MENU_VISIBLE) {
        int steps = haptic_mode_item_to_steps(g_haptic_mode_sel);
        haptic_set_num_steps(steps);
        g_haptic_mode_menu = HAPTIC_MODE_MENU_HIDDEN;
        LOG_INF("Haptic mode applied: %s", haptic_mode_item_name(g_haptic_mode_sel));
        log_menu_state("mode applied");
        return;
    }

    if (g_ui_mode == UI_MODE_EDIT) {
        confirm_edit_and_send();
        return;
    }

    if (g_level == MENU_LEVEL_ROOT) {
        switch (g_root_sel) {
        case ROOT_STATUS_MENU:
            log_menu_state("confirm status");
            return;
        case ROOT_CONTROL:
            g_level = MENU_LEVEL_CONTROL;
            g_ctrl_sel = CTRL_HOME_ALL;
            log_menu_state("enter submenu");
            return;
        case ROOT_TEMPERATURE:
            g_level = MENU_LEVEL_TEMPERATURE;
            g_temp_sel = TEMP_EXTRUDER;
            log_menu_state("enter submenu");
            return;
        case ROOT_FILAMENT:
            g_level = MENU_LEVEL_FILAMENT;
            g_filament_sel = FILAMENT_PREHEAT_PLA;
            log_menu_state("enter submenu");
            return;
        case ROOT_SD_CARD:
            g_level = MENU_LEVEL_SD_CARD;
            log_menu_state("enter submenu");
            return;
        case ROOT_CALIBRATION:
            g_level = MENU_LEVEL_CALIBRATION;
            g_calib_sel = CALIB_Z;
            log_menu_state("enter submenu");
            return;
        case ROOT_MMU:
            g_level = MENU_LEVEL_MMU;
            g_mmu_sel = MMU_HOME;
            log_menu_state("enter submenu");
            return;
        case ROOT_ZOFFSET:
            enter_edit_mode(EDIT_ZOFFSET);
            return;
        case ROOT_MACROS:
            g_level = MENU_LEVEL_MACROS;
            g_macros_sel = MACROS_SHEET;
            log_menu_state("enter submenu");
            return;
        case ROOT_PRINTING:
            g_level = MENU_LEVEL_PRINTING;
            g_printing_sel = PRINTING_PAUSE;
            log_menu_state("enter submenu");
            return;
        default:
            return;
        }
    }

    if (g_level == MENU_LEVEL_CONTROL) {
        switch (g_ctrl_sel) {
        case CTRL_HOME_ALL:
            send_simple_gcode("home:all");
            log_menu_state("home all");
            return;
        case CTRL_X:
            g_level = MENU_LEVEL_CONTROL_X;
            g_ctrl_axis = 0;
            g_ctrl_axis_sel = CTRL_AXIS_HOME;
            log_menu_state("enter submenu");
            return;
        case CTRL_Y:
            g_level = MENU_LEVEL_CONTROL_Y;
            g_ctrl_axis = 1;
            g_ctrl_axis_sel = CTRL_AXIS_HOME;
            log_menu_state("enter submenu");
            return;
        case CTRL_Z:
            g_level = MENU_LEVEL_CONTROL_Z;
            g_ctrl_axis = 2;
            g_ctrl_axis_sel = CTRL_AXIS_HOME;
            log_menu_state("enter submenu");
            return;
        case CTRL_E:
            enter_edit_mode(EDIT_POS_E);
            return;
        case CTRL_FAN:
            enter_edit_mode(EDIT_FAN);
            return;
        case CTRL_MOTORS_OFF:
            send_simple_gcode("motors:off");
            log_menu_state("motors off");
            return;
        default:
            return;
        }
    }

    if (g_level == MENU_LEVEL_CONTROL_X ||
        g_level == MENU_LEVEL_CONTROL_Y ||
        g_level == MENU_LEVEL_CONTROL_Z) {
        static const char * const home_cmds[] = {"home:x", "home:y", "home:z"};
        static const edit_target_t move_targets[] = {EDIT_POS_X, EDIT_POS_Y, EDIT_POS_Z};
        int ax = g_ctrl_axis;
        switch (g_ctrl_axis_sel) {
        case CTRL_AXIS_HOME:
            send_simple_gcode(home_cmds[ax]);
            log_menu_state("axis home");
            return;
        case CTRL_AXIS_MOVE:
            enter_edit_mode(move_targets[ax]);
            return;
        default:
            return;
        }
    }

    if (g_level == MENU_LEVEL_TEMPERATURE) {
        switch (g_temp_sel) {
        case TEMP_EXTRUDER:  enter_edit_mode(EDIT_TEMP_EXTRUDER); return;
        case TEMP_BED:       enter_edit_mode(EDIT_TEMP_BED);      return;
        case TEMP_COOL_DOWN: send_simple_gcode("cool:down"); log_menu_state("cool down"); return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_FILAMENT) {
        switch (g_filament_sel) {
        case FILAMENT_PREHEAT_PLA:  send_simple_gcode("filament:preheat:pla");  log_menu_state("preheat pla");  return;
        case FILAMENT_PREHEAT_PETG: send_simple_gcode("filament:preheat:petg"); log_menu_state("preheat petg"); return;
        case FILAMENT_LOAD:         send_simple_gcode("filament:load");         log_menu_state("load");         return;
        case FILAMENT_UNLOAD:       send_simple_gcode("filament:unload");       log_menu_state("unload");       return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_CALIBRATION) {
        switch (g_calib_sel) {
        case CALIB_Z:           send_simple_gcode("calib:z");           log_menu_state("calib z");           return;
        case CALIB_BED_MESH:    send_simple_gcode("calib:bed_mesh");    log_menu_state("calib bed mesh");    return;
        case CALIB_FIRST_LAYER: send_simple_gcode("calib:first_layer"); log_menu_state("calib first layer"); return;
        case CALIB_PROBE:       send_simple_gcode("calib:probe");       log_menu_state("calib probe");       return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_MMU) {
        switch (g_mmu_sel) {
        case MMU_HOME:    send_simple_gcode("mmu:home");   log_menu_state("mmu home");   return;
        case MMU_RESUME:  send_simple_gcode("mmu:resume"); log_menu_state("mmu resume"); return;
        case MMU_LOCATE_SELECTOR:
            g_level = MENU_LEVEL_MMU_LOCATE;
            g_mmu_tool_sel = MMU_T0;
            log_menu_state("enter submenu");
            return;
        case MMU_SET_TOOL:
            g_level = MENU_LEVEL_MMU_SET_TOOL;
            g_mmu_tool_sel = MMU_T0;
            log_menu_state("enter submenu");
            return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_MMU_LOCATE) {
        char cmd[16];
        (void)snprintf(cmd, sizeof(cmd), "mmu:locate:%d", (int)g_mmu_tool_sel);
        send_text_gcode(cmd);
        g_level = MENU_LEVEL_MMU;
        log_menu_state("mmu locate selected");
        return;
    }

    if (g_level == MENU_LEVEL_MMU_SET_TOOL) {
        send_tool(g_mmu_tool_sel);
        g_level = MENU_LEVEL_MMU;
        log_menu_state("mmu tool selected");
        return;
    }

    if (g_level == MENU_LEVEL_MACROS) {
        switch (g_macros_sel) {
        case MACROS_SHEET:
            g_level = MENU_LEVEL_SHEET;
            g_sheet_sel = SHEET_CUSTOM0;
            log_menu_state("enter submenu");
            return;
        case MACROS_LIGHT:
            toggle_light();
            log_menu_state("light toggled");
            return;
        case MACROS_FAKE_POSITION:
            send_simple_gcode("fake:position");
            log_menu_state("fake position");
            return;
        default: return;
        }
    }

    if (g_level == MENU_LEVEL_SHEET) {
        send_sheet(g_sheet_sel);
        g_level = MENU_LEVEL_MACROS;
        g_macros_sel = MACROS_SHEET;
        log_menu_state("sheet selected");
        return;
    }

    if (g_level == MENU_LEVEL_PRINTING) {
        switch (g_printing_sel) {
        case PRINTING_PAUSE: send_simple_gcode("print:pause"); log_menu_state("print pause"); return;
        case PRINTING_FLOW:  enter_edit_mode(EDIT_FLOW);  return;
        case PRINTING_SPEED: enter_edit_mode(EDIT_SPEED); return;
        default: return;
        }
    }
}

static void on_back_action(void)
{
    ui_display_flash_back();

    if (g_haptic_mode_menu == HAPTIC_MODE_MENU_VISIBLE) {
        g_haptic_mode_menu = HAPTIC_MODE_MENU_HIDDEN;
        log_menu_state("mode cancel");
        return;
    }

    if (g_ui_mode == UI_MODE_EDIT) {
        g_ui_mode = UI_MODE_NAV;
        g_edit_target = EDIT_NONE;
        log_menu_state("cancel edit");
        return;
    }

    switch (g_level) {
    case MENU_LEVEL_ROOT:
        LOG_INF("MENU back (root) | mode=NAV | level=ROOT");
        k_work_cancel_delayable(&g_idle_disp_work);
        request_status_mode();
        return;
    case MENU_LEVEL_CONTROL:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_CONTROL;
        log_menu_state("back");
        return;
    case MENU_LEVEL_CONTROL_X:
        g_level = MENU_LEVEL_CONTROL;
        g_ctrl_sel = CTRL_X;
        log_menu_state("back");
        return;
    case MENU_LEVEL_CONTROL_Y:
        g_level = MENU_LEVEL_CONTROL;
        g_ctrl_sel = CTRL_Y;
        log_menu_state("back");
        return;
    case MENU_LEVEL_CONTROL_Z:
        g_level = MENU_LEVEL_CONTROL;
        g_ctrl_sel = CTRL_Z;
        log_menu_state("back");
        return;
    case MENU_LEVEL_TEMPERATURE:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_TEMPERATURE;
        log_menu_state("back");
        return;
    case MENU_LEVEL_FILAMENT:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_FILAMENT;
        log_menu_state("back");
        return;
    case MENU_LEVEL_SD_CARD:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_SD_CARD;
        log_menu_state("back");
        return;
    case MENU_LEVEL_CALIBRATION:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_CALIBRATION;
        log_menu_state("back");
        return;
    case MENU_LEVEL_MMU:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_MMU;
        log_menu_state("back");
        return;
    case MENU_LEVEL_MMU_LOCATE:
        g_level = MENU_LEVEL_MMU;
        g_mmu_sel = MMU_LOCATE_SELECTOR;
        log_menu_state("back");
        return;
    case MENU_LEVEL_MMU_SET_TOOL:
        g_level = MENU_LEVEL_MMU;
        g_mmu_sel = MMU_SET_TOOL;
        log_menu_state("back");
        return;
    case MENU_LEVEL_MACROS:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_MACROS;
        log_menu_state("back");
        return;
    case MENU_LEVEL_SHEET:
        g_level = MENU_LEVEL_MACROS;
        g_macros_sel = MACROS_SHEET;
        log_menu_state("back");
        return;
    case MENU_LEVEL_PRINTING:
        g_level = MENU_LEVEL_ROOT;
        g_root_sel = ROOT_PRINTING;
        log_menu_state("back");
        return;
    default:
        return;
    }
}

static void on_virtual_click(int dir)
{
    if (g_haptic_mode_menu == HAPTIC_MODE_MENU_VISIBLE) {
        if (dir >= 0) {
            on_back_action();
        } else {
            on_confirm_action();
        }
        return;
    }

    if (g_pending_click_dir != 0 && g_pending_click_dir == dir) {
        k_work_cancel_delayable(&g_click_eval_work);
        g_pending_click_dir = 0;
        g_haptic_mode_menu = HAPTIC_MODE_MENU_VISIBLE;
        g_haptic_mode_sel = haptic_mode_steps_to_item(haptic_get_num_steps());
        log_menu_state("mode menu open (double click)");
        return;
    }

    if (g_pending_click_dir != 0 && g_pending_click_dir != dir) {
        k_work_cancel_delayable(&g_click_eval_work);
        if (g_pending_click_dir >= 0) {
            on_back_action();
        } else {
            on_confirm_action();
        }
        g_pending_click_dir = 0;
    }

    g_pending_click_dir = dir;
    k_work_reschedule(&g_click_eval_work, K_MSEC(DOUBLE_CLICK_WINDOW_MS));
}

static void click_eval_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    int dir = g_pending_click_dir;
    g_pending_click_dir = 0;
    if (dir >= 0) {
        on_back_action();
    } else {
        on_confirm_action();
    }
}

void remote_control_init(void)
{
    int err = ble_commands_init();
    if (err != 0) {
        LOG_ERR("BLE init failed (%d)", err);
    }

    g_level          = MENU_LEVEL_ROOT;
    g_root_sel       = ROOT_CONTROL;
    g_ctrl_sel       = CTRL_HOME_ALL;
    g_ctrl_axis_sel  = CTRL_AXIS_HOME;
    g_ctrl_axis      = 0;
    g_temp_sel       = TEMP_EXTRUDER;
    g_filament_sel   = FILAMENT_PREHEAT_PLA;
    g_calib_sel      = CALIB_Z;
    g_mmu_sel        = MMU_HOME;
    g_mmu_tool_sel   = MMU_T0;
    g_macros_sel     = MACROS_SHEET;
    g_sheet_sel      = SHEET_CUSTOM0;
    g_printing_sel   = PRINTING_PAUSE;
    g_ui_mode     = UI_MODE_NAV;
    g_edit_target = EDIT_NONE;
    g_light_on    = false;
    g_haptic_mode_menu = HAPTIC_MODE_MENU_HIDDEN;
    g_haptic_mode_sel = haptic_mode_steps_to_item(haptic_get_num_steps());
    g_pending_click_dir = 0;
    atomic_set(&g_menu_refresh_pending, 0);
    atomic_set(&g_status_mode_pending, 0);
    k_work_init_delayable(&g_click_eval_work, click_eval_work_handler);
    k_work_init_delayable(&g_idle_disp_work, idle_disp_work_handler);
    haptic_set_step_callback(on_knob_step);
    haptic_set_virtual_click_callback(on_virtual_click);
    LOG_INF("Remote menu init (single click=confirm/back, double click=mode menu)");
    log_menu_state("init");
    /* Keep boot/wake default on status screen. init logging queues a menu
     * refresh, so clear it explicitly. */
    atomic_set(&g_menu_refresh_pending, 0);
    ui_display_set_mode(UI_DISP_STATUS);
}

void remote_control_tick(void)
{
    if (atomic_cas(&g_status_mode_pending, 1, 0)) {
        ui_display_set_mode(UI_DISP_STATUS);
    }

    /* Called periodically from main loop. Push current BLE state to status
     * screen so the display reflects live printer data. */
    struct ble_printer_state st;
    if (ble_printer_state_get(&st)) {
        /* Fill in fields not yet provided by BLE (future expansion) */
        if (st.fan_pct == 0.0f && !st.printing) {
            /* keep zero */
        }
        if (st.feed_rate_pct == 0.0f) {
            st.feed_rate_pct = 100.0f; /* default */
        }
        if (st.status_msg[0] == '\0') {
            if (st.printing) {
                (void)strncpy(st.status_msg, "Printing", sizeof(st.status_msg) - 1);
            } else {
                (void)strncpy(st.status_msg, "Ready", sizeof(st.status_msg) - 1);
            }
        }
    } else {
        /* No BLE data: send invalid state so display shows "No printer data" */
        (void)memset(&st, 0, sizeof(st));
        st.valid = false;
    }
    ui_display_update_status(&st);

    if (atomic_cas(&g_menu_refresh_pending, 1, 0)) {
        display_push_menu();
    }
}
