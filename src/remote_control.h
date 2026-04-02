#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

typedef enum {
    MENU_LEVEL_ROOT,
    MENU_LEVEL_POSITION,
    MENU_LEVEL_TEMPERATURE,
} menu_level_t;

typedef enum {
    ROOT_STATUS_MENU,
    ROOT_POSITION,
    ROOT_TEMPERATURE,
    ROOT_ITEM_COUNT,
} root_item_t;

typedef enum {
    POS_BACK,
    POS_X,
    POS_Y,
    POS_Z,
    POS_E,
    POS_ITEM_COUNT,
} position_item_t;

typedef enum {
    TEMP_BACK,
    TEMP_EXTRUDER,
    TEMP_BED,
    TEMP_ITEM_COUNT,
} temperature_item_t;

void remote_control_init(void);

#endif /* REMOTE_CONTROL_H */