#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/input/input.h>
#include <zephyr/devicetree.h>

/* Menu button callback - define in your .c file:
 *
 * static void menu_button_cb(struct input_event *evt, void *user_data)
 * {
 *     if (evt->code == INPUT_KEY_ENTER && evt->value == 1) {
 *         // button pressed
 *     }
 * }
 * INPUT_CALLBACK_DEFINE(NULL, menu_button_cb, NULL);
 */

typedef enum {
    PARAM_POSITION,
    PARAM_TEMPERATURE,
    IDLE
} remote_param_t;

typedef struct {
    remote_param_t param;
    int32_t        value;
} ble_command_t;

typedef struct {
    int            cursor;
    bool           editing;
    remote_param_t selected;
} menu_state_t;

#endif /* REMOTE_CONTROL_H */