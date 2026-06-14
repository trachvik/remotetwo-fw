#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "haptic.h"
#include "remote_control.h"
#include "drivers/tmag5170_sensor.h"
#include "ui_display.h"
#include "BLE_commands.h"

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

	/* Sequence copied verbatim from the proven `main` branch, which ran the
	 * SPI display + TMAG encoder + DRV8311 motor + BLE all together on the
	 * nRF5340 DK:
	 *
	 *   1) Start the BLE radio FIRST.
	 *   2) Wait 300 ms so the RF front-end start-up current surge settles.
	 *      Without this the surge drops PVDD below the DRV8311H UVLO threshold
	 *      (latching a permanent fault) and disturbs the MIC2288 OLED boost,
	 *      leaving the display white and the TMAG5170 reading 0 mT.
	 *   3) Only THEN bring up the display + motor driver + encoder.
	 *
	 * No DC/DC Kconfig tricks — just the settle delay, exactly like main. */
	ble_commands_init();
	k_msleep(300);

	/* Supply now stable: bring up display and haptic. */
	if (ui_display_init() == 0) {
		(void)ui_display_show_hello_remote();
	}

	haptic_init(&motor, (bldc_driver_t *)&driver, (sensor_t *)&encoder);

	/* Finish UI/menu init (BLE already up). */
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
