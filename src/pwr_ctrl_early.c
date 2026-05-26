/*
 * pwr_ctrl_early.c
 *
 * Enable the display logic power rail (TPS62740 LOAD / pwr_ctrl, P0.30)
 * before the SSD1309 driver initialises (POST_KERNEL, priority 85).
 *
 * Without this the SSD1309 Zephyr driver runs its SPI init sequence while
 * the display logic supply is still off, leaving the display uninitialised.
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

static int pwr_ctrl_early_enable(void)
{
	static const struct gpio_dt_spec pwr =
		GPIO_DT_SPEC_GET(DT_NODELABEL(pwr_ctrl), gpios);

	if (!gpio_is_ready_dt(&pwr)) {
		return -ENODEV;
	}
	gpio_pin_configure_dt(&pwr, GPIO_OUTPUT_ACTIVE);
	/* Allow the logic supply to stabilise before SSD1309 init */
	k_msleep(5);
	return 0;
}

SYS_INIT(pwr_ctrl_early_enable, POST_KERNEL, 70);
