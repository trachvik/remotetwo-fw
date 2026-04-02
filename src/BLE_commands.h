#ifndef BLE_COMMANDS_H
#define BLE_COMMANDS_H

int ble_commands_init(void);
void ble_send_gcode(const char *cmd);

#endif /* BLE_COMMANDS_H */
