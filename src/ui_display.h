#ifndef UI_DISPLAY_H
#define UI_DISPLAY_H

#include "remote_control.h"
#include "BLE_commands.h"

/* --- Display mode --- */
typedef enum {
    UI_DISP_STATUS,   /* status screen (shown after 5 s inactivity) */
    UI_DISP_MENU,     /* scrollable navigation menu */
    UI_DISP_EDIT,     /* value edit screen */
} ui_disp_mode_t;

/* Call after every menu state change to push new content to display. */
void ui_display_update_menu(menu_level_t level, int selected, int item_count,
                            const char * const *item_names, const char *header);

/* Push printer status data to the status screen. Call whenever BLE state updates. */
void ui_display_update_status(const struct ble_printer_state *state);

/* Push an edit-mode screen showing the parameter name and current value string. */
void ui_display_update_edit(const char *param_name, const char *value_str);

/* Visual press feedback: inverts the confirm/back button (or selected item) for 100 ms. */
void ui_display_flash_confirm(void);
void ui_display_flash_back(void);

/* Switch between status screen and menu screen. */
void ui_display_set_mode(ui_disp_mode_t mode);

int  ui_display_init(void);
int  ui_display_show_hello_remote(void);
void ui_display_process(void);

#endif /* UI_DISPLAY_H */
