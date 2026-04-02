#include "remote_control.h"
#include "BLE_commands.h"
#include "haptic.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(remote_control, LOG_LEVEL_DBG);

static remote_param_t current_param = IDLE;

static remote_param_t param_wrap_step(remote_param_t param, int dir)
{
    int p = (int)param + dir;
    if (p > (int)IDLE) {
        p = (int)PARAM_POSITION;
    }
    if (p < (int)PARAM_POSITION) {
        p = (int)IDLE;
    }
    return (remote_param_t)p;
}

static void on_knob_step(int dir)
{
    current_param = param_wrap_step(current_param, dir);
    LOG_INF("Knob step dir=%d, new param=%d", dir, current_param);
    ble_send_gcode(current_param);
}

void remote_control_init(void)
{
    current_param = PARAM_POSITION;
    haptic_set_step_callback(on_knob_step);
    LOG_INF("Remote control init, param=%d", current_param);
}

