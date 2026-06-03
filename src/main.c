#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "haptic.h"
#include "remote_control.h"
#include "drivers/tmag5170_sensor.h"
#include "ui_display.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* Device instances */
static struct tmag5170_device encoder;
static bldc_driver_3pwm_t driver;
static bldc_motor_t motor;

int main(void)
{
	LOG_INF("System started");

	/* Display before BLE/haptic */
	if (ui_display_init() == 0) {
		(void)ui_display_show_hello_remote();
	}

	/* Initialize BLE */
	remote_control_init();

	/* Initialize haptic motor */
	haptic_init(&motor, (bldc_driver_t *)&driver, (sensor_t *)&encoder);

	while (1) {
		remote_control_tick();
		ui_display_process();
		k_msleep(15);
	}
}
