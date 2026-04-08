#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

typedef enum {
    MENU_LEVEL_ROOT,
    MENU_LEVEL_POSITION,
    MENU_LEVEL_TEMPERATURE,
    MENU_LEVEL_TOOL,
    MENU_LEVEL_MACROS,
    MENU_LEVEL_SHEET,
} menu_level_t;

typedef enum {
    ROOT_STATUS_MENU,
    ROOT_POSITION,
    ROOT_TEMPERATURE,
    ROOT_TOOL,
    ROOT_ZOFFSET,
    ROOT_MACROS,
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

typedef enum {
    TOOL_BACK,
    TOOL_T0,
    TOOL_T1,
    TOOL_T2,
    TOOL_T3,
    TOOL_T4,
    TOOL_T5,
    TOOL_T6,
    TOOL_T7,
    TOOL_T8,
    TOOL_ITEM_COUNT,
} tool_item_t;

typedef enum {
    MACROS_BACK,
    MACROS_SHEET,
    MACROS_LIGHT,
    MACROS_ITEM_COUNT,
} macros_item_t;

typedef enum {
    SHEET_BACK,
    SHEET_CUSTOM0,
    SHEET_CUSTOM1,
    SHEET_CUSTOM2,
    SHEET_ITEM_COUNT,
} sheet_item_t;

void remote_control_init(void);

#endif /* REMOTE_CONTROL_H */