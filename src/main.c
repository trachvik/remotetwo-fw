#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
  #include "haptic.h"
  #include "remote_control.h"
 #include "ui_display.h"
 #include "drivers/as5048a.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* Device instances */
 static struct as5048a_device encoder;
 static bldc_driver_6pwm_t driver;
 static bldc_motor_t motor;

// /* Global pointer for sensor wrapper */
 struct as5048a_device *g_as5048a = &encoder;

int main(void)
{
	LOG_INF("System started");

	/* Initialize BLE early so nRF Connect can discover the device immediately. */
	remote_control_init();

	/* Initialize haptic motor */
	 haptic_init(&motor, (bldc_driver_t*)&driver, (sensor_t*)&encoder);

	/* Initialize display backend and render startup text using LVGL. */
	if (ui_display_init() == 0) {
		(void)ui_display_show_hello_remote();
	}

	while (1) {
		k_msleep(1);
	}
}
