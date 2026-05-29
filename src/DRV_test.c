/*
 * DRV8311H 3PWM open-loop driver verification
 *
 * Exercises the DRV8311H gate driver via bldc_driver_3pwm HAL in voltage-mode
 * open loop — no encoder, no current sensing.  Follows the SimpleFOC
 * velocityOpenloop + setPhaseVoltage(Uq, Ud=0, angle_el) pattern with
 * SINE_PWM and centred modulation.
 *
 * INL logic level (DRV8311H datasheet, Table 1 – 3PWM mode):
 *   INL = HIGH → low-side gates enabled, complementary PWM (normal operation)
 *   INL = LOW  → all low-side gates forced off, motor coasts (Hi-Z)
 * bldc_driver_3pwm_init_hw() asserts INL HIGH as the first GPIO action.
 *
 * Slew rate: 75 V/µs is set by hardware (IDRIVEN/IDRIVEP resistors).
 * No firmware register writes required.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "drivers/bldc_driver_3pwm.h"

/* M_PI is a GNU extension and is not exposed by picolibc in strict C99 mode. */
#define DRV_PI  3.14159265358979323846f
#define DRV_2PI (2.0f * DRV_PI)

LOG_MODULE_REGISTER(drv_test, LOG_LEVEL_INF);

/* ── Motor / test parameters ─────────────────────────────────────────────── */
#define POLE_PAIRS          7       /* adjust to actual motor                  */
#define VOLTAGE_SUPPLY      3.75f   /* V, actual VM on PCB (measure with DMM!) */
#define OPEN_LOOP_VOLTAGE   1.5f    /* V, Uq ≤ Vdc/2 = 1.875 V                */
#define TARGET_VEL_MECH    25.0f    /* rad/s mechanical shaft velocity target  */

/* 2 kHz control loop driven by k_busy_wait. */
#define LOOP_PERIOD_US     500U
#define DT                 (LOOP_PERIOD_US * 1.0e-6f)   /* fixed timestep, s   */
#define LOG_EVERY_N        2000U                         /* ≈ 1 log line/second */

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/** Wrap any angle into [0, 2π]. */
static inline float norm_angle(float a)
{
    float r = fmodf(a, DRV_2PI);
    return (r < 0.0f) ? r + DRV_2PI : r;
}

/**
 * Sinusoidal phase voltages centred at Vdc/2.
 *
 * Mirrors SimpleFOC setPhaseVoltage(Uq, Ud=0, angle_el)
 * with SINE_PWM and modulation_centered = true:
 *
 *   Inverse Park (Ud = 0):
 *     Uα = -Uq · sin(θ)
 *     Uβ =  Uq · cos(θ)
 *
 *   Inverse Clarke:
 *     Ua =  Uα
 *     Ub = -Uα/2 + (√3/2)·Uβ
 *     Uc = -Uα/2 - (√3/2)·Uβ    (= -(Ua+Ub), since Ia+Ib+Ic = 0)
 *
 *   Centre shift (add Vdc/2):
 *     Ua += Vdc/2,  Ub += Vdc/2,  Uc += Vdc/2
 *
 * Resulting voltage swing: [Vdc/2 - Uq, Vdc/2 + Uq] ⊂ [0, Vdc]. ✓
 */
static void set_phase_voltage(bldc_driver_3pwm_t *drv, float uq, float angle_el)
{
    const float mid  = drv->voltage_power_supply * 0.5f;
    const float u_al = -uq * sinf(angle_el);          /* Uα (inverse Park)    */
    const float u_be =  uq * cosf(angle_el);          /* Uβ (inverse Park)    */

    const float ua = mid + u_al;
    const float ub = mid + (-u_al * 0.5f + 0.8660254f * u_be);
    const float uc = mid + (-u_al * 0.5f - 0.8660254f * u_be);

    bldc_driver_3pwm_set_pwm(drv, ua, ub, uc);
}

/* ── Entry point ─────────────────────────────────────────────────────────── */

int main(void)
{
    static bldc_driver_3pwm_t drv;

    LOG_INF("=== DRV8311H OPEN-LOOP TEST ===");
    LOG_INF("This is build_drv, not the UI/BLE/encoder firmware");
    LOG_INF("Target: %.1f rad/s mech, Uq = %.1f V, %d pole pairs",
            (double)TARGET_VEL_MECH, (double)OPEN_LOOP_VOLTAGE, POLE_PAIRS);

    bldc_driver_3pwm_init_struct(&drv);
    drv.voltage_power_supply = VOLTAGE_SUPPLY;
    drv.voltage_limit        = VOLTAGE_SUPPLY;
    drv.pwm_frequency        = 25000;

    if (bldc_driver_3pwm_init_hw(&drv) != DRIVER_INIT_OK) {
        LOG_ERR("bldc_driver_3pwm_init_hw failed — check overlay / GPIOs");
        return -1;
    }

    bldc_driver_3pwm_enable(&drv);
    LOG_INF("Driver ready: nSLEEP=HIGH, INL=HIGH");

    /* Give the rotor a starting preference before entering open-loop spin. */
    LOG_INF("Aligning rotor...");
    for (uint32_t i = 0; i < 1000U; ++i) {
        set_phase_voltage(&drv, OPEN_LOOP_VOLTAGE, 0.0f);
        k_busy_wait(1000U);
    }
    LOG_INF("Alignment complete, starting spin");

    const float vel_elec = TARGET_VEL_MECH * (float)POLE_PAIRS;  /* elec. rad/s */
    float shaft_angle    = 0.0f;
    uint32_t iter        = 0;

    while (1) {
        /* Open-loop angle integration (fixed timestep). */
        shaft_angle = norm_angle(shaft_angle + TARGET_VEL_MECH * DT);

        /* Electrical angle = mechanical angle × pole pairs. */
        const float angle_el = norm_angle(shaft_angle * (float)POLE_PAIRS);

        /* Apply sinusoidal phase voltages. */
        set_phase_voltage(&drv, OPEN_LOOP_VOLTAGE, angle_el);

        /* Periodic log — approximately once per second. */
        if (++iter >= LOG_EVERY_N) {
            iter = 0;
            LOG_INF("shaft=%.3f rad  elec=%.3f rad  ω_el=%.1f rad/s"
                    "  dc=[%.3f %.3f %.3f]",
                    (double)shaft_angle,
                    (double)angle_el,
                    (double)vel_elec,
                    (double)drv.dc_a,
                    (double)drv.dc_b,
                    (double)drv.dc_c);
        }

        k_busy_wait(LOOP_PERIOD_US);
    }

    return 0;
}
