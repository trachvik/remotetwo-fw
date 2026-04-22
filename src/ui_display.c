#include "ui_display.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <lvgl.h>
#include <lvgl_zephyr.h>

LOG_MODULE_REGISTER(ui_display, LOG_LEVEL_INF);

int ui_display_init(void)
{
    /*
     * Zephyr LVGL integration initialises lv_init() and registers the
     * SSD1306/SSD1309 display driver automatically via SYS_INIT before
     * main() runs.  Nothing to do here.
     */
    LOG_INF("ui_display ready (Zephyr LVGL integration active)");
    return 0;
}

int ui_display_show_hello_remote(void)
{
    lv_display_t *disp = lv_display_get_default();
    if (disp == NULL) {
        LOG_ERR("No LVGL default display");
        return -ENODEV;
    }

    lvgl_lock();

    lv_obj_t *scr = lv_scr_act();

    lv_obj_clean(scr);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Hello Remote");
    lv_obj_center(label);

    lv_refr_now(disp);

    lvgl_unlock();

    return 0;
}

void ui_display_process(void)
{
    /* Zephyr LVGL integration runs its own worker/timer; no manual pump here. */
}
