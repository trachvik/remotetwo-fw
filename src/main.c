#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* Alias led0 z devicetree */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* Timer callback – volana v preruseni jednou za sekundu */
static void timer_handler(struct k_timer *timer)
{
	gpio_pin_toggle_dt(&led);
}

K_TIMER_DEFINE(blink_timer, timer_handler, NULL);

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return -1;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	/* Spustit timer: prvni expirace za 1 s, perioda 1 s */
	k_timer_start(&blink_timer, K_SECONDS(1), K_SECONDS(1));

	return 0;
}
