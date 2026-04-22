#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ui_display.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

int main(void)
{
	LOG_INF("System started");

	if (ui_display_init() == 0) {
		(void)ui_display_show_hello_remote();
	}

	while (1) {
		ui_display_process();
		k_msleep(50);
	}
}
