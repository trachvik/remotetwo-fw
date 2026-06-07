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

/* nFAULT interrupt bookkeeping */
static struct gpio_callback fault_cb_data;

static void fault_isr(const struct device *dev, struct gpio_callback *cb,
                      uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    /* nFAULT is active-low, so a falling edge = fault asserted */
    LOG_ERR("DRV8311H nFAULT asserted! Check overcurrent / overtemp / UVLO.");
}

/* ------------------------------------------------------------------ */
/* Internal helper: write duty cycles directly to PWM hardware        */
/* ------------------------------------------------------------------ */
static void write_duty_cycles(uint32_t period_ns, float dc_a, float dc_b, float dc_c)
{
    uint32_t pa = (uint32_t)(_CONSTRAIN(dc_a, 0.0f, 1.0f) * (float)period_ns);
    uint32_t pb = (uint32_t)(_CONSTRAIN(dc_b, 0.0f, 1.0f) * (float)period_ns);
    uint32_t pc = (uint32_t)(_CONSTRAIN(dc_c, 0.0f, 1.0f) * (float)period_ns);

    int r0 = pwm_set_dt(&pwm_ina, period_ns, pa);
    int r1 = pwm_set_dt(&pwm_inb, period_ns, pb);
    int r2 = pwm_set_dt(&pwm_inc, period_ns, pc);

    if (r0 || r1 || r2) {
        static bool logged_once = false;
        if (!logged_once) {
            logged_once = true;
            LOG_ERR("pwm_set_dt failed: INA=%d INB=%d INC=%d (period=%u)",
                    r0, r1, r2, period_ns);
        }
    }
}

/* Cached period so set_pwm doesn't recompute every call */
static uint32_t g_period_ns = 0;

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

    /* Configure nSLEEP – reset pulse (LOW→HIGH) clears any latched faults */
    if (!gpio_is_ready_dt(&gpio_sleep)) {
        LOG_ERR("nSLEEP GPIO not ready");
        return DRIVER_INIT_FAILED;
    }
    if (gpio_pin_configure_dt(&gpio_sleep, GPIO_OUTPUT_INACTIVE | GPIO_INPUT) != 0) {
        LOG_ERR("Failed to configure nSLEEP as output");
        return DRIVER_INIT_FAILED;
    }
    k_sleep(K_MSEC(2));
    gpio_pin_set_dt(&gpio_sleep, 1);
    k_sleep(K_MSEC(5));
    LOG_INF("nSLEEP reset pulse done");

    /* Configure INL – drive HIGH (low-side bridge permanently enabled) */
    if (!gpio_is_ready_dt(&gpio_inl)) {
        LOG_ERR("INL GPIO not ready");
        return DRIVER_INIT_FAILED;
    }
    if (gpio_pin_configure_dt(&gpio_inl, GPIO_OUTPUT_ACTIVE | GPIO_INPUT) != 0) {
        LOG_ERR("Failed to configure INL as output");
        return DRIVER_INIT_FAILED;
    }

    /* Configure nFAULT – input with interrupt on falling edge (fault asserted) */
    if (!gpio_is_ready_dt(&gpio_fault)) {
        LOG_WRN("nFAULT GPIO not ready – fault detection disabled");
    } else {
        gpio_pin_configure_dt(&gpio_fault, GPIO_INPUT);

        int raw = gpio_pin_get_raw(gpio_fault.port, gpio_fault.pin);
        LOG_INF("nFAULT raw pin level = %d  (1=HIGH=OK, 0=LOW=fault)", raw);
        if (raw == 0) {
            LOG_ERR("DRV8311H nFAULT is LOW (fault asserted)! Check HW fault, OCP, open-load, or VCP charge-pump.");
        } else {
            LOG_INF("nFAULT = HIGH (OK), IC ready");
        }

        gpio_pin_interrupt_configure_dt(&gpio_fault, GPIO_INT_EDGE_FALLING);
        gpio_init_callback(&fault_cb_data, fault_isr,
                           BIT(gpio_fault.pin));
        gpio_add_callback(gpio_fault.port, &fault_cb_data);
        LOG_INF("nFAULT interrupt on P%d.%02d",
                gpio_fault.port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1,
                gpio_fault.pin);
    }

    LOG_INF("INL  raw=%d  (P%d.%02d, expect HIGH)",
            gpio_pin_get_raw(gpio_inl.port, gpio_inl.pin),
            gpio_inl.port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1,
            gpio_inl.pin);
    LOG_INF("nSLEEP raw=%d  (P%d.%02d, expect HIGH)",
            gpio_pin_get_raw(gpio_sleep.port, gpio_sleep.pin),
            gpio_sleep.port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1,
            gpio_sleep.pin);

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

void bldc_driver_3pwm_get_status(int *nfault, int *nsleep, int *inl)
{
    if (nfault) {
        *nfault = gpio_is_ready_dt(&gpio_fault)
                      ? gpio_pin_get_raw(gpio_fault.port, gpio_fault.pin) : -1;
    }
    if (nsleep) {
        *nsleep = gpio_is_ready_dt(&gpio_sleep)
                      ? gpio_pin_get_raw(gpio_sleep.port, gpio_sleep.pin) : -1;
    }
    if (inl) {
        *inl = gpio_is_ready_dt(&gpio_inl)
                   ? gpio_pin_get_raw(gpio_inl.port, gpio_inl.pin) : -1;
    }
}

int bldc_driver_3pwm_reassert_enable(int *nsleep_after, int *inl_after,
                                     int *sleep_cfg_rc, int *inl_cfg_rc,
                                     int *sleep_set_rc)
{
    int rc_cfg_s = -100, rc_cfg_i = -100, rc_set_s = -100;

    if (gpio_is_ready_dt(&gpio_inl)) {
        rc_cfg_i = gpio_pin_configure_dt(&gpio_inl, GPIO_OUTPUT_ACTIVE);
    }
    if (gpio_is_ready_dt(&gpio_sleep)) {
        rc_cfg_s = gpio_pin_configure_dt(&gpio_sleep, GPIO_OUTPUT_ACTIVE);
        rc_set_s = gpio_pin_set_dt(&gpio_sleep, 1);
    }

    k_busy_wait(2000U); /* let the pin settle / DRV wake */

    if (inl_cfg_rc)   *inl_cfg_rc   = rc_cfg_i;
    if (sleep_cfg_rc) *sleep_cfg_rc = rc_cfg_s;
    if (sleep_set_rc) *sleep_set_rc = rc_set_s;
    if (nsleep_after) {
        *nsleep_after = gpio_is_ready_dt(&gpio_sleep)
                          ? gpio_pin_get_raw(gpio_sleep.port, gpio_sleep.pin) : -1;
    }
    if (inl_after) {
        *inl_after = gpio_is_ready_dt(&gpio_inl)
                       ? gpio_pin_get_raw(gpio_inl.port, gpio_inl.pin) : -1;
    }
    return (rc_cfg_i == 0 && rc_cfg_s == 0 && rc_set_s == 0) ? 0 : -1;
}
