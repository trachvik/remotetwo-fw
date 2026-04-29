#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

typedef enum {
    MENU_LEVEL_ROOT,
    MENU_LEVEL_CONTROL,
    MENU_LEVEL_CONTROL_X,
    MENU_LEVEL_CONTROL_Y,
    MENU_LEVEL_CONTROL_Z,
    MENU_LEVEL_TEMPERATURE,
    MENU_LEVEL_FILAMENT,
    MENU_LEVEL_SD_CARD,
    MENU_LEVEL_CALIBRATION,
    MENU_LEVEL_MMU,
    MENU_LEVEL_MMU_LOCATE,
    MENU_LEVEL_MMU_SET_TOOL,
    MENU_LEVEL_MACROS,
    MENU_LEVEL_SHEET,
    MENU_LEVEL_PRINTING,
} menu_level_t;

typedef enum {
    ROOT_STATUS_MENU,
    ROOT_CONTROL,
    ROOT_TEMPERATURE,
    ROOT_FILAMENT,
    ROOT_SD_CARD,
    ROOT_CALIBRATION,
    ROOT_MMU,
    ROOT_ZOFFSET,
    ROOT_MACROS,
    ROOT_PRINTING,
    ROOT_ITEM_COUNT,
} root_item_t;

typedef enum {
    CTRL_HOME_ALL,
    CTRL_X,
    CTRL_Y,
    CTRL_Z,
    CTRL_E,
    CTRL_FAN,
    CTRL_MOTORS_OFF,
    CTRL_ITEM_COUNT,
} control_item_t;

typedef enum {
    CTRL_AXIS_HOME,
    CTRL_AXIS_MOVE,
    CTRL_AXIS_ITEM_COUNT,
} control_axis_item_t;

typedef enum {
    TEMP_EXTRUDER,
    TEMP_BED,
    TEMP_COOL_DOWN,
    TEMP_ITEM_COUNT,
} temperature_item_t;

typedef enum {
    FILAMENT_PREHEAT_PLA,
    FILAMENT_PREHEAT_PETG,
    FILAMENT_LOAD,
    FILAMENT_UNLOAD,
    FILAMENT_ITEM_COUNT,
} filament_item_t;

typedef enum {
    CALIB_Z,
    CALIB_BED_MESH,
    CALIB_FIRST_LAYER,
    CALIB_PROBE,
    CALIB_ITEM_COUNT,
} calibration_item_t;

typedef enum {
    MMU_HOME,
    MMU_RESUME,
    MMU_LOCATE_SELECTOR,
    MMU_SET_TOOL,
    MMU_ITEM_COUNT,
} mmu_item_t;

typedef enum {
    MMU_T0,
    MMU_T1,
    MMU_T2,
    MMU_T3,
    MMU_T4,
    MMU_T5,
    MMU_T6,
    MMU_T7,
    MMU_T8,
    MMU_TOOL_ITEM_COUNT,
} mmu_tool_item_t;

typedef enum {
    MACROS_SHEET,
    MACROS_LIGHT,
    MACROS_FAKE_POSITION,
    MACROS_ITEM_COUNT,
} macros_item_t;

typedef enum {
    SHEET_CUSTOM0,
    SHEET_CUSTOM1,
    SHEET_CUSTOM2,
    SHEET_ITEM_COUNT,
} sheet_item_t;

typedef enum {
    PRINTING_PAUSE,
    PRINTING_FLOW,
    PRINTING_SPEED,
    PRINTING_ITEM_COUNT,
} printing_item_t;

void remote_control_init(void);
void remote_control_tick(void); /* call periodically from main loop */

#endif /* REMOTE_CONTROL_H */