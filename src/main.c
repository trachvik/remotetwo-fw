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

	/* Initialize haptic motor FIRST and let it finish completely.
	 * The FOC calibration is timing-sensitive and floods the USB-CDC log;
	 * running BLE (bt_enable + net-core HCI IPC) concurrently starves it and
	 * leaves the motor uninitialised. BLE is started afterwards from
	 * remote_control_init(). */
	haptic_init(&motor, (bldc_driver_t *)&driver, (sensor_t *)&encoder);

	/* Register knob callbacks and start BLE (off the critical path). */
	remote_control_init();

	uint32_t heartbeat = 0;

	while (1) {
		remote_control_tick();
		ui_display_process();

		if ((heartbeat++ % 200) == 0U) {
			/* The USB-CDC console is dead for ~16 s during USB (re)enumeration
			 * after boot, which swallows every haptic_init() log line. Drain the
			 * diagnostic buffer SLOWLY here — one line per heartbeat (~3 s) — so
			 * the milestones land in the window where the console is alive again.
			 * Also report the live motor status so we can tell whether FOC init
			 * actually succeeded. */
			static int diag_printed = 0;
			if (diag_printed < haptic_diag_count()) {
				LOG_INF("HAPTIC %s", haptic_diag_get_line(diag_printed));
				diag_printed++;
			}
			LOG_INF("App alive | mstatus=%d zea=%.3f tgt=%.3f loop=%u cum=%.3f steps=%u",
				motor.motor_status,
				(double)motor.zero_electric_angle,
				(double)motor.target,
				haptic_loop_count(),
				(double)haptic_dbg_cumulative_angle(),
				haptic_step_fire_count());
		}

		k_msleep(15);
	}
}
