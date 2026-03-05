#include <zephyr/kernel.h>
#include "haptic.h"
#include "drivers/as5048a.h"

/* Device instances */
static struct as5048a_device encoder;
static bldc_driver_6pwm_t driver;
static bldc_motor_t motor;

/* Global pointer for sensor wrapper */
struct as5048a_device *g_as5048a = &encoder;

int main(void)
{
	/* Initialize haptic motor */
	haptic_init(&motor, (bldc_driver_t*)&driver, (sensor_t*)&encoder);

	/* haptic_loop is now driven by a 10 kHz timer thread in haptic.c */
	while (1) {
		k_sleep(K_FOREVER);
	}
}
