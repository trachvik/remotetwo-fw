#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

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

void remote_control_init(void);
void remote_control_process(void);

#endif /* REMOTE_CONTROL_H */