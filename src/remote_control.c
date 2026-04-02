#include "remote_control.h"
#include "BLE_commands.h"
#include "haptic.h"
#include <math.h>
#include <stdio.h>
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
} edit_target_t;

typedef struct {
    edit_target_t target;
    const char *name;
    const char *token;
    float value_min;
    float value_max;
    float value_default;
    float step_fine;
    float step_mid;
    float step_coarse;
} param_desc_t;

static const param_desc_t g_params[] = {
    { EDIT_POS_X, "X", "PX", -200.0f, 200.0f, 0.0f, 0.1f, 0.5f, 1.0f },
    { EDIT_POS_Y, "Y", "PY", -200.0f, 200.0f, 0.0f, 0.1f, 0.5f, 1.0f },
    { EDIT_POS_Z, "Z", "PZ", -200.0f, 200.0f, 0.0f, 0.1f, 0.5f, 1.0f },
    { EDIT_POS_E, "E", "PE", -200.0f, 200.0f, 0.0f, 0.1f, 0.5f, 1.0f },
    { EDIT_TEMP_EXTRUDER, "extruder", "TE", 0.0f, 300.0f, 200.0f, 0.5f, 1.0f, 5.0f },
    { EDIT_TEMP_BED, "bed", "TB", 0.0f, 120.0f, 60.0f, 0.5f, 1.0f, 5.0f },
};

static menu_level_t g_level = MENU_LEVEL_ROOT;
static root_item_t g_root_sel = ROOT_STATUS_MENU;
static position_item_t g_pos_sel = POS_BACK;
static temperature_item_t g_temp_sel = TEMP_BACK;
static ui_mode_t g_ui_mode = UI_MODE_NAV;
static edit_target_t g_edit_target = EDIT_NONE;
static float g_edit_value = 0.0f;
static int64_t g_last_edit_step_ms = 0;
static int64_t g_btn2_press_ms = 0;

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

static float current_edit_step(const param_desc_t *desc)
{
    int64_t now_ms = k_uptime_get();
    int64_t dt_ms = now_ms - g_last_edit_step_ms;
    g_last_edit_step_ms = now_ms;

    if (dt_ms <= 120) {
        return desc->step_coarse;
    }
    if (dt_ms <= 300) {
        return desc->step_mid;
    }
    return desc->step_fine;
}

static const char *level_name(menu_level_t level)
{
    switch (level) {
    case MENU_LEVEL_ROOT:
        return "ROOT";
    case MENU_LEVEL_POSITION:
        return "POSITION";
    case MENU_LEVEL_TEMPERATURE:
        return "TEMPERATURE";
    default:
        return "UNKNOWN";
    }
}

static const char *root_item_name(root_item_t item)
{
    switch (item) {
    case ROOT_STATUS_MENU:
        return "status menu";
    case ROOT_POSITION:
        return "position";
    case ROOT_TEMPERATURE:
        return "temperature";
    default:
        return "?";
    }
}

static const char *position_item_name(position_item_t item)
{
    switch (item) {
    case POS_BACK:
        return "back";
    case POS_X:
        return "X";
    case POS_Y:
        return "Y";
    case POS_Z:
        return "Z";
    case POS_E:
        return "E";
    default:
        return "?";
    }
}

static const char *temperature_item_name(temperature_item_t item)
{
    switch (item) {
    case TEMP_BACK:
        return "back";
    case TEMP_EXTRUDER:
        return "extruder";
    case TEMP_BED:
        return "bed";
    default:
        return "?";
    }
}

static const char *current_selection_name(void)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        const param_desc_t *desc = find_param_desc(g_edit_target);
        return (desc != NULL) ? desc->name : "edit";
    }

    switch (g_level) {
    case MENU_LEVEL_ROOT:
        return root_item_name(g_root_sel);
    case MENU_LEVEL_POSITION:
        return position_item_name(g_pos_sel);
    case MENU_LEVEL_TEMPERATURE:
        return temperature_item_name(g_temp_sel);
    default:
        return "?";
    }
}

static void log_menu_state(const char *reason)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        LOG_INF("MENU %s | mode=EDIT | level=%s | selected=%s | value=%.1f",
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

static void enter_edit_mode(edit_target_t target)
{
    const param_desc_t *desc = find_param_desc(target);
    if (desc == NULL) {
        return;
    }

    g_ui_mode = UI_MODE_EDIT;
    g_edit_target = target;
    g_edit_value = desc->value_default;
    g_last_edit_step_ms = k_uptime_get();
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

    char cmd[20];
    (void)snprintf(cmd, sizeof(cmd), "%s:%.1f", desc->token, (double)g_edit_value);
    send_text_gcode(cmd);

    g_ui_mode = UI_MODE_NAV;
    g_edit_target = EDIT_NONE;
    log_menu_state("exit edit");
}

static void on_knob_step(int dir)
{
    if (g_ui_mode == UI_MODE_EDIT) {
        const param_desc_t *desc = find_param_desc(g_edit_target);
        if (desc == NULL) {
            return;
        }

        float step = current_edit_step(desc);
        g_edit_value = clampf(g_edit_value + ((float)dir * step),
                              desc->value_min, desc->value_max);
        g_edit_value = roundf(g_edit_value * 10.0f) / 10.0f;
        LOG_INF("Edit knob dir=%d step=%.2f", dir, (double)step);
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
    }

    LOG_INF("Knob dir=%d", dir);
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
        case POS_X:
            enter_edit_mode(EDIT_POS_X);
            return;
        case POS_Y:
            enter_edit_mode(EDIT_POS_Y);
            return;
        case POS_Z:
            enter_edit_mode(EDIT_POS_Z);
            return;
        case POS_E:
            enter_edit_mode(EDIT_POS_E);
            return;
        default:
            return;
        }
    }

    if (g_level == MENU_LEVEL_TEMPERATURE) {
        switch (g_temp_sel) {
        case TEMP_BACK:
            g_level = MENU_LEVEL_ROOT;
            g_root_sel = ROOT_TEMPERATURE;
            log_menu_state("back");
            return;
        case TEMP_EXTRUDER:
            enter_edit_mode(EDIT_TEMP_EXTRUDER);
            return;
        case TEMP_BED:
            enter_edit_mode(EDIT_TEMP_BED);
            return;
        default:
            return;
        }
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

static void btn2_input_cb(struct input_event *evt, void *user_data)
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

INPUT_CALLBACK_DEFINE(NULL, btn2_input_cb, NULL);

void remote_control_init(void)
{
    int err = ble_commands_init();
    if (err != 0) {
        LOG_ERR("BLE init failed (%d)", err);
    }

    g_level = MENU_LEVEL_ROOT;
    g_root_sel = ROOT_STATUS_MENU;
    g_pos_sel = POS_BACK;
    g_temp_sel = TEMP_BACK;
    g_ui_mode = UI_MODE_NAV;
    g_edit_target = EDIT_NONE;
    haptic_set_step_callback(on_knob_step);
    LOG_INF("Remote menu init (btn2=confirm, long-press=back)");
    log_menu_state("init");
}

