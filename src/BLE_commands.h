#ifndef BLE_COMMANDS_H
#define BLE_COMMANDS_H

#include "remote_control.h"

int ble_commands_init(void);
void ble_send_gcode(remote_param_t param);

#endif /* BLE_COMMANDS_H */
