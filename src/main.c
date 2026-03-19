#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
// #include "haptic.h"
// #include "drivers/as5048a.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* Device instances */
// static struct as5048a_device encoder;
// static bldc_driver_6pwm_t driver;
// static bldc_motor_t motor;

// /* Global pointer for sensor wrapper */
// struct as5048a_device *g_as5048a = &encoder;

int main(void)
{
	LOG_INF("System started");

	/* Initialize haptic motor */
	// haptic_init(&motor, (bldc_driver_t*)&driver, (sensor_t*)&encoder);

	int counter = 0;

	while (1) {
		//LOG_INF("Loop iteration: %d", counter++);
		// k_sleep(K_MSEC(1000));
		k_sleep(K_MSEC(1));
	}
}
