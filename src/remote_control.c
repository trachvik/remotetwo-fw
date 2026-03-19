#include "remote_control.h"
#include "BLE_commands.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(remote_control, LOG_LEVEL_DBG);

remote_param_t current_param = IDLE;

// button callback to cycle through parameters
static void menu_button_cb(struct input_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);

    if (evt->code == INPUT_KEY_ENTER && evt->value == 1) {
        if (current_param < IDLE) {
            current_param++;
        } else {
            current_param = PARAM_POSITION;
        }
        LOG_INF("Menu button pressed, new param: %d", current_param);
        ble_send_gcode(current_param);
    }
}

INPUT_CALLBACK_DEFINE(NULL, menu_button_cb, NULL);

