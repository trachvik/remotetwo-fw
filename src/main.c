#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
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

static void wait_for_usb_console_ready(void)
{
#if DT_HAS_CHOSEN(zephyr_console)
	const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!device_is_ready(console_dev)) {
		return;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		uint32_t dtr = 0;

		/* Wait max 3 s for terminal DTR – do not block indefinitely so that
		 * mcumgr SMP commands can reach the app even without a terminal. */
		for (int i = 0; i < 120; i++) {
			if ((uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr) == 0) && dtr) {
				break;
			}
			k_msleep(25);
		}
	}
#endif
}

int main(void)
{
	wait_for_usb_console_ready();
	LOG_INF("System started, OTA update ready");

	/* Display before BLE/haptic */
	if (ui_display_init() == 0) {
		(void)ui_display_show_hello_remote();
	}

	/* Initialize BLE */
	remote_control_init();

	/* Initialize haptic motor */
	haptic_init(&motor, (bldc_driver_t *)&driver, (sensor_t *)&encoder);

	uint32_t heartbeat = 0;

	while (1) {
		remote_control_tick();
		ui_display_process();

		if ((heartbeat++ % 200) == 0U) {
			LOG_INF("App alive");
		}

		k_msleep(15);
	}
}
