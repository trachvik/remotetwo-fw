#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <arm_math.h>
/* #include "haptic.h" */          /* disabled: no encoder yet */
#include "remote_control.h"
#include "ui_display.h"
#include "drivers/bldc_driver_3pwm.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* MIC2288 boost converter enable (P0.28) — must be HIGH at startup */
static const struct gpio_dt_spec g_mic2288 =
	GPIO_DT_SPEC_GET(DT_NODELABEL(mic2288_mos), gpios);

/* -----------------------------------------------------------------------
 * Open-loop voltage control parameters
 * Tune OPENLOOP_RPM and OPENLOOP_VAMP to suit the motor.
 * OPENLOOP_VSUPPLY must match the actual supply voltage [V].
 * OPENLOOP_POLE_PAIRS must match the motor winding (11 for this motor).
 * ----------------------------------------------------------------------- */
#define OPENLOOP_VSUPPLY     3.7f   /* V  — LiPo battery voltage            */
#define OPENLOOP_VAMP        1.0f   /* V  — bootstrap-safe: max duty=(1.85+1.0)/3.7=77% */
#define OPENLOOP_RPM        30.0f   /* mechanical RPM                       */
#define OPENLOOP_POLE_PAIRS 11      /* electrical = mechanical * pole_pairs */

/* Loop period (ms) — main loop sleeps this long each iteration */
#define LOOP_MS             1

static bldc_driver_3pwm_t driver;
/* static bldc_motor_t motor; */   /* disabled: used only by haptic */

int main(void)
{
	LOG_INF("System started");

	/* MIC2288 boost converter — enable immediately */
	gpio_pin_configure_dt(&g_mic2288, GPIO_OUTPUT_ACTIVE);

	/* Display */
	if (ui_display_init() == 0) {
		(void)ui_display_show_hello_remote();
	}

	/* BLE */
	remote_control_init();

	/* ---- Open-loop driver init ---- */
	bldc_driver_3pwm_init_struct(&driver);
	driver.voltage_power_supply = OPENLOOP_VSUPPLY;
	driver.voltage_limit        = OPENLOOP_VSUPPLY;
	if (bldc_driver_3pwm_init_hw(&driver) != DRIVER_INIT_OK) {
		LOG_ERR("DRV8311H init failed");
	} else {
		bldc_driver_3pwm_enable(&driver);
		LOG_INF("Open-loop start: %.0f RPM, Vamp=%.1f V",
			(double)OPENLOOP_RPM, (double)OPENLOOP_VAMP);
	}

	/* Electrical angular velocity [rad/s] */
	const float omega_e = (OPENLOOP_RPM / 60.0f) * 2.0f * PI
			      * (float)OPENLOOP_POLE_PAIRS;
	const float dt      = LOOP_MS * 0.001f;
	const float voff    = OPENLOOP_VSUPPLY * 0.5f;  /* DC offset = Vsupply/2 */

	/* ---- Alignment pulse ----
	 * Hold rotor at theta = PI/2 (phase A maximum) for 500 ms.
	 * If motor is connected and driver outputs are working, you MUST
	 * hear a click or feel the rotor snap to position.
	 * Silence here = motor disconnected or DRV8311H not driving output.
	 */
	float va_al = voff + OPENLOOP_VAMP * arm_sin_f32(PI / 2.0f);
	float vb_al = voff + OPENLOOP_VAMP * arm_sin_f32(PI / 2.0f - 2.0f * PI / 3.0f);
	float vc_al = voff + OPENLOOP_VAMP * arm_sin_f32(PI / 2.0f + 2.0f * PI / 3.0f);
	LOG_INF("Alignment: va=%.2f vb=%.2f vc=%.2f — listen for click!",
		(double)va_al, (double)vb_al, (double)vc_al);
	bldc_driver_3pwm_set_pwm(&driver, va_al, vb_al, vc_al);
	k_msleep(500);
	LOG_INF("Alignment done, starting sweep");

	float theta_e       = PI / 2.0f;  /* start from aligned position */
	int diag_counter    = 0;

	while (1) {
		/* ---- Open-loop sinusoidal voltage output ---- */
		float va = voff + OPENLOOP_VAMP * arm_sin_f32(theta_e);
		float vb = voff + OPENLOOP_VAMP * arm_sin_f32(theta_e - 2.0f * PI / 3.0f);
		float vc = voff + OPENLOOP_VAMP * arm_sin_f32(theta_e + 2.0f * PI / 3.0f);

		if (bldc_driver_3pwm_is_fault(&driver)) {
			/* Fault active — motor disabled by ISR; do not output. */
		} else {
			bldc_driver_3pwm_set_pwm(&driver, va, vb, vc);
			theta_e += omega_e * dt;
			if (theta_e >= 2.0f * PI) {
				theta_e -= 2.0f * PI;
			}
		}

		/* Print state once per second for diagnostics */
		if (++diag_counter >= (1000 / LOOP_MS)) {
			diag_counter = 0;
			bool fault = bldc_driver_3pwm_is_fault(&driver);
			LOG_INF("OL va=%.2f vb=%.2f vc=%.2f theta=%.2f fault=%d",
				(double)va, (double)vb, (double)vc,
				(double)theta_e, fault ? 1 : 0);
		}

		/* haptic_loop(&motor); */   /* disabled: no encoder */
		remote_control_tick();
		ui_display_process();
		k_msleep(LOOP_MS);
	}
}
