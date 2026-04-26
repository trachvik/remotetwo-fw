#ifndef BLE_COMMANDS_H
#define BLE_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

struct ble_printer_state {
	bool valid;
	bool can_move;
	bool printing;

	bool homed_x;
	bool homed_y;
	bool homed_z;

	int tool;

	float pos_x;
	float pos_y;
	float pos_z;
	float pos_e;

	float temp_e;        /* extruder temperature [°C] */
	float temp_b;        /* bed temperature [°C] */

	float fan_pct;       /* part cooling fan 0-100 % */
	float feed_rate_pct; /* feed rate override 0-999 % */
	float progress_pct;  /* print progress 0-100 % */
	uint32_t print_secs; /* elapsed print time [s] */
	char status_msg[20]; /* short printer status string, e.g. "Ready" */
};

struct ble_last_ack {
	bool valid;
	uint16_t id;
	bool ok;
	char reason[20];
};

int ble_commands_init(void);
void ble_send_gcode(const char *cmd);
bool ble_printer_state_get(struct ble_printer_state *out_state);
bool ble_printer_can_move(void);
bool ble_last_ack_get(struct ble_last_ack *out_ack);

#endif /* BLE_COMMANDS_H */
