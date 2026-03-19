#include "BLE_commands.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_commands, LOG_LEVEL_DBG);

void ble_send_gcode(remote_param_t param)
{
    switch (param) {
    case PARAM_POSITION:
        LOG_INF("GCODE: position change");
        break;
    case PARAM_TEMPERATURE:
        LOG_INF("GCODE: temp change");
        break;
    default:
        LOG_INF("GCODE: idle / no command");
        break;
    }
}
