#include "bldc_driver_3pwm.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bldc_drv3pwm, LOG_LEVEL_INF);

#define _CONSTRAIN(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* ------------------------------------------------------------------ */
/* Device-tree resources                                               */
/* All hardware pins are defined in the board overlay under            */
/* the 'bldc_drv' node.                                                */
/* ------------------------------------------------------------------ */
#define DRV_NODE DT_NODELABEL(bldc_drv)

/* PWM channels: INA = idx 0, INB = idx 1, INC = idx 2 */
static const struct pwm_dt_spec pwm_ina = PWM_DT_SPEC_GET_BY_IDX(DRV_NODE, 0);
static const struct pwm_dt_spec pwm_inb = PWM_DT_SPEC_GET_BY_IDX(DRV_NODE, 1);
static const struct pwm_dt_spec pwm_inc = PWM_DT_SPEC_GET_BY_IDX(DRV_NODE, 2);

/* GPIO: INL (low-side enable, active-high), nSLEEP (active-high), nFAULT (active-low) */
static const struct gpio_dt_spec gpio_inl   = GPIO_DT_SPEC_GET(DRV_NODE, inl_gpios);
static const struct gpio_dt_spec gpio_sleep = GPIO_DT_SPEC_GET(DRV_NODE, sleep_gpios);
static const struct gpio_dt_spec gpio_fault = GPIO_DT_SPEC_GET(DRV_NODE, fault_gpios);

/* ------------------------------------------------------------------ */
/* Internal helper: write duty cycles directly to PWM hardware        */
/* ------------------------------------------------------------------ */

/* Cached period so set_pwm doesn't recompute every call */
static uint32_t g_period_ns = 0;

static void write_duty_cycles(uint32_t period_ns, float dc_a, float dc_b, float dc_c)
{
    int ret;
    uint32_t pa = (uint32_t)(_CONSTRAIN(dc_a, 0.0f, 1.0f) * (float)period_ns);
    uint32_t pb = (uint32_t)(_CONSTRAIN(dc_b, 0.0f, 1.0f) * (float)period_ns);
    uint32_t pc = (uint32_t)(_CONSTRAIN(dc_c, 0.0f, 1.0f) * (float)period_ns);

    ret = pwm_set_dt(&pwm_ina, period_ns, pa);
    if (ret) { LOG_ERR("pwm_ina (ch0 P0.07) set failed: %d", ret); }
    ret = pwm_set_dt(&pwm_inb, period_ns, pb);
    if (ret) { LOG_ERR("pwm_inb (ch1 P1.05) set failed: %d", ret); }
    ret = pwm_set_dt(&pwm_inc, period_ns, pc);
    if (ret) { LOG_ERR("pwm_inc (ch2 P0.22) set failed: %d", ret); }
}

/* nFAULT interrupt bookkeeping */
static struct gpio_callback fault_cb_data;

/* Set to true when nFAULT is asserted (falling edge or level check). */
static volatile bool g_fault_active = false;

static void fault_isr(const struct device *dev, struct gpio_callback *cb,
                      uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    /* nFAULT fell LOW: disable motor immediately, then inform application. */
    g_fault_active = true;
    write_duty_cycles(g_period_ns, 0.0f, 0.0f, 0.0f);
    LOG_ERR("DRV8311H nFAULT asserted — motor disabled. Check OCP/OTP/UVLO.");
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void bldc_driver_3pwm_init_struct(bldc_driver_3pwm_t *driver)
{
    if (driver == NULL) {
        return;
    }
    driver->voltage_power_supply = DEF_POWER_SUPPLY;
    driver->voltage_limit        = DEF_POWER_SUPPLY;
    driver->pwm_frequency        = 25000;
    driver->dc_a                 = 0.0f;
    driver->dc_b                 = 0.0f;
    driver->dc_c                 = 0.0f;
    driver->initialized          = false;
}

int bldc_driver_3pwm_init_hw(bldc_driver_3pwm_t *driver)
{
    if (driver == NULL) {
        return DRIVER_INIT_FAILED;
    }

    /* Sanity-check voltage limit */
    if (driver->voltage_limit <= 0.0f ||
        driver->voltage_limit > driver->voltage_power_supply) {
        driver->voltage_limit = driver->voltage_power_supply;
    }

    /* Verify PWM channels */
    if (!pwm_is_ready_dt(&pwm_ina) ||
        !pwm_is_ready_dt(&pwm_inb) ||
        !pwm_is_ready_dt(&pwm_inc)) {
        LOG_ERR("One or more PWM channels not ready");
        return DRIVER_INIT_FAILED;
    }

    /* Compute and cache PWM period */
    long freq = (driver->pwm_frequency > 0) ? driver->pwm_frequency : 25000;
    g_period_ns = (uint32_t)(1000000000UL / (unsigned long)freq);

    /* All channels to 0 % duty cycle */
    write_duty_cycles(g_period_ns, 0.0f, 0.0f, 0.0f);

    /* Configure nSLEEP – reset pulse to clear any latched fault, then HIGH */
    if (!gpio_is_ready_dt(&gpio_sleep)) {
        LOG_ERR("nSLEEP GPIO not ready");
        return DRIVER_INIT_FAILED;
    }
    /* Pulse nSLEEP LOW for tRST to clear any latched fault that occurred
     * during power-on (e.g. CPUV latched before firmware ran, when PWM pins
     * were floating and nSLEEP was pulled HIGH by a board resistor).
     * tRST spec: min 10 µs, max 65 µs (DRV8311H datasheet). */
    gpio_pin_configure_dt(&gpio_sleep, GPIO_OUTPUT_INACTIVE); /* nSLEEP LOW */
    k_usleep(20);  /* 20 µs — within tRST window */
    gpio_pin_configure_dt(&gpio_sleep, GPIO_OUTPUT_ACTIVE);   /* nSLEEP HIGH */
    LOG_INF("nSLEEP: reset pulse (20us LOW) -> HIGH (P1.01) – latched faults cleared");
    /* Wait for charge pump to reach VCPUV threshold (tWAKE typ=1ms, max=3ms). */
    k_msleep(5); /* 5 ms > tWAKE_max=3 ms, gives margin */

    /* Configure INL – drive HIGH (low-side bridge permanently enabled) */
    if (!gpio_is_ready_dt(&gpio_inl)) {
        LOG_ERR("INL GPIO not ready");
        return DRIVER_INIT_FAILED;
    }
    gpio_pin_configure_dt(&gpio_inl, GPIO_OUTPUT_ACTIVE);     /* active-high → HIGH */
    LOG_INF("INL HIGH (P0.18) – low-side enabled");

    /* Configure nFAULT – input with interrupt on falling edge (fault asserted) */
    if (!gpio_is_ready_dt(&gpio_fault)) {
        LOG_WRN("nFAULT GPIO not ready – fault detection disabled");
    } else {
        /* nFAULT is open-drain: DRV8311H only pulls LOW on fault, otherwise
         * the pin is high-Z.  Without a pull-up the floating pin reads as 0
         * (apparent fault) even when the chip is healthy.  Enable the nRF5340
         * internal pull-up (~13 kΩ) so the pin sits HIGH when no fault. */
        gpio_pin_configure_dt(&gpio_fault, GPIO_INPUT | GPIO_PULL_UP);

        /* Read initial fault state.
         * gpio_pin_get_dt: logical level (ACTIVE_LOW → 0=no-fault, 1=fault).
         * gpio_pin_get_raw: physical level (1=pin HIGH=no-fault, 0=pin LOW=fault).
         * Both are logged so polarity is unambiguous. */
        int fault_logical = gpio_pin_get_dt(&gpio_fault);
        int fault_raw     = gpio_pin_get_raw(gpio_fault.port, gpio_fault.pin);
        LOG_INF("nFAULT logical=%d raw=%d (raw=1→pin HIGH→no fault)",
                fault_logical, fault_raw);
        if (fault_raw == 0) {
            LOG_ERR("DRV8311H nFAULT is LOW (pin=GND) – chip is in FAULT state!");
        }

        gpio_pin_interrupt_configure_dt(&gpio_fault, GPIO_INT_EDGE_FALLING);
        gpio_init_callback(&fault_cb_data, fault_isr,
                           BIT(gpio_fault.pin));
        gpio_add_callback(gpio_fault.port, &fault_cb_data);
        LOG_INF("nFAULT interrupt configured on P0.%02d", gpio_fault.pin);
    }

    driver->initialized = true;
    LOG_INF("DRV8311H 3PWM driver ready: freq=%ld Hz, period=%u ns, Vsupply=%.1f V",
            freq, g_period_ns, (double)driver->voltage_power_supply);

    return DRIVER_INIT_OK;
}

void bldc_driver_3pwm_enable(bldc_driver_3pwm_t *driver)
{
    if (driver == NULL || !driver->initialized) {
        return;
    }
    /* Zero duty cycles – motor at rest, bridge stays active (INL already HIGH) */
    write_duty_cycles(g_period_ns, 0.0f, 0.0f, 0.0f);
    driver->dc_a = 0.0f;
    driver->dc_b = 0.0f;
    driver->dc_c = 0.0f;
}

void bldc_driver_3pwm_disable(bldc_driver_3pwm_t *driver)
{
    if (driver == NULL || !driver->initialized) {
        return;
    }
    /* Zero duty cycles – motor coasts, INL stays HIGH */
    write_duty_cycles(g_period_ns, 0.0f, 0.0f, 0.0f);
    driver->dc_a = 0.0f;
    driver->dc_b = 0.0f;
    driver->dc_c = 0.0f;
}

bool bldc_driver_3pwm_is_fault(bldc_driver_3pwm_t *driver)
{
    if (driver == NULL || !driver->initialized) {
        return false;
    }
    /* Read physical pin level — raw=0 means pin is LOW = fault asserted.
     * Reflect current state (not latched) so motor recovers when fault clears.
     * ISR still zeroes PWM immediately on falling edge. */
    g_fault_active = (gpio_pin_get_raw(gpio_fault.port, gpio_fault.pin) == 0);
    return g_fault_active;
}

void bldc_driver_3pwm_set_pwm(bldc_driver_3pwm_t *driver,
                               float ua, float ub, float uc)
{
    if (driver == NULL || !driver->initialized) {
        return;
    }

    /* Clamp voltages to [0, voltage_limit] */
    ua = _CONSTRAIN(ua, 0.0f, driver->voltage_limit);
    ub = _CONSTRAIN(ub, 0.0f, driver->voltage_limit);
    uc = _CONSTRAIN(uc, 0.0f, driver->voltage_limit);

    /* Convert to duty cycle [0, 1] */
    driver->dc_a = ua / driver->voltage_power_supply;
    driver->dc_b = ub / driver->voltage_power_supply;
    driver->dc_c = uc / driver->voltage_power_supply;

    write_duty_cycles(g_period_ns, driver->dc_a, driver->dc_b, driver->dc_c);
}
