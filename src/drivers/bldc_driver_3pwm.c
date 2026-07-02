#include "bldc_driver_3pwm.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_pwm.h>

LOG_MODULE_REGISTER(bldc_drv3pwm, LOG_LEVEL_INF);

#define _CONSTRAIN(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* ------------------------------------------------------------------ */
/* Device-tree resources                                               */
/* All hardware pins are defined in the board overlay under            */
/* the 'bldc_drv' node.                                                */
/* ------------------------------------------------------------------ */
#define DRV_NODE DT_NODELABEL(bldc_drv)

/* PWM channels: INA = idx 0, INB = idx 1, INC = idx 2
 * Kept only for the initial Zephyr PWM initialisation call that triggers
 * pinctrl (pin routing). Actual PWM is driven by nrfx_pwm directly. */
static const struct pwm_dt_spec pwm_ina = PWM_DT_SPEC_GET_BY_IDX(DRV_NODE, 0);
static const struct pwm_dt_spec pwm_inb = PWM_DT_SPEC_GET_BY_IDX(DRV_NODE, 1);
static const struct pwm_dt_spec pwm_inc = PWM_DT_SPEC_GET_BY_IDX(DRV_NODE, 2);

/* GPIO: INL (low-side enable, active-high), nSLEEP (active-high), nFAULT (active-low) */
static const struct gpio_dt_spec gpio_inl   = GPIO_DT_SPEC_GET(DRV_NODE, inl_gpios);
static const struct gpio_dt_spec gpio_sleep = GPIO_DT_SPEC_GET(DRV_NODE, sleep_gpios);
static const struct gpio_dt_spec gpio_fault = GPIO_DT_SPEC_GET(DRV_NODE, fault_gpios);

/* ------------------------------------------------------------------ */
/* Center-aligned PWM via nrfx_pwm (NRF_PWM_MODE_UP_AND_DOWN)        */
/*                                                                     */
/* Why center-aligned?                                                 */
/* Edge-aligned (COUNT_UP, normal polarity) places ALL switching edges */
/* at t=0 (PWMPERIODEND). This creates a single large simultaneous     */
/* transient on the CSA outputs that lasts ~28 us and contaminates     */
/* most of the 40 us period, leaving only ~6 us at the very end for   */
/* reliable current sensing (not enough for 3 channels).               */
/*                                                                     */
/* Center-aligned (UP_AND_DOWN) places each phase's edges individually */
/* spread across the upswing/downswing, and the t=0 boundary (counter */
/* bottom) has NO simultaneous switching. The all-low-sides-on window  */
/* is at the start of the period (0 .. min_compare*2/16MHz us) and is */
/* quiet, giving at least 3x5=15 us for the 3-channel SAADC scan.     */
/*                                                                     */
/* COUNTERTOP = 320 -> period = 2*320 / 16 MHz = 40 us = 25 kHz.     */
/* Compare value for duty D = (1-D)*320 (INVERTED polarity, so the    */
/* high-side FET is ON when counter >= compare, i.e. near the TOP).   */
/* The low-side FET is ON in the EDGES (near counter=0), which is     */
/* where the SAADC samples (ISENSE_SAMPLE_FRACTION ~0.1 -> t~4 us).  */
/* ------------------------------------------------------------------ */
#define PWM_CENTER_ALIGNED_COUNTERTOP 320u   /* 16 MHz / (2*320) = 25 kHz */

/* Sequence buffer in RAM: 4 x uint16_t for NRF_PWM_LOAD_INDIVIDUAL mode.
 * [0]=INA  [1]=INB  [2]=INC  [3]=unused.
 * Each entry: bit[15]=1 (INVERTED polarity) | bits[14:0]=compare ticks.
 * The PWM DMA reads from this buffer every period (pointer stored in SEQ[0].PTR).
 * Updating the buffer takes effect on the next period without restarting the
 * peripheral. No nrfx_pwm.h / nrfx_pwm_t needed - direct register access only,
 * consistent with how current_sense.c handles TIMER2. */
#define PWM_SEQ_SIZE 4u
static uint16_t m_seq_buf[PWM_SEQ_SIZE];

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
/* Internal helper: update center-aligned duty cycles via nrfx_pwm   */
/* ------------------------------------------------------------------ */
/* Convert a phase voltage fraction [0,1] to an nrfx_pwm sequence
 * compare value for UP_AND_DOWN mode with INVERTED polarity.
 * The high-side FET is ON when counter >= compare (near the TOP).
 * compare = (1-D) * COUNTERTOP, bit[15] = 1 (inverted polarity). */
static inline uint16_t duty_to_compare(float duty)
{
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;
    uint16_t cmp = (uint16_t)((1.0f - duty) * (float)PWM_CENTER_ALIGNED_COUNTERTOP);
    return cmp | 0x8000u; /* bit[15] = INVERTED polarity */
}

static void write_duty_cycles(uint32_t period_ns, float dc_a, float dc_b, float dc_c)
{
    ARG_UNUSED(period_ns); /* period is fixed by COUNTERTOP */

    m_seq_buf[0] = duty_to_compare(dc_a);
    m_seq_buf[1] = duty_to_compare(dc_b);
    m_seq_buf[2] = duty_to_compare(dc_c);
    m_seq_buf[3] = 0x8000u; /* unused 4th channel: 0% duty, inverted */
    /* The nrfx_pwm peripheral reads from m_seq_buf every period (it stores the
     * pointer from the initial simple_playback call and re-reads on each loop).
     * No need to restart the sequence - the new values take effect next period. */
}

/* Cached period so API callers don't need to recompute */
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

    /* Trigger Zephyr PWM driver once per channel to activate pinctrl
     * (routes the three GPIO pins to PWM_OUT0/1/2). After this we take over
     * the peripheral with direct register writes - no further Zephyr PWM calls.
     * Directly writing NRF_PWM0 registers (instead of using nrfx_pwm) avoids
     * creating a second nrfx_pwm_t instance that would conflict with the one
     * the Zephyr driver owns and corrupt the nrfx interrupt routing. */
    if (!pwm_is_ready_dt(&pwm_ina) ||
        !pwm_is_ready_dt(&pwm_inb) ||
        !pwm_is_ready_dt(&pwm_inc)) {
        LOG_ERR("One or more PWM channels not ready");
        return DRIVER_INIT_FAILED;
    }
    (void)pwm_set_dt(&pwm_ina, PWM_KHZ(25), 0);
    (void)pwm_set_dt(&pwm_inb, PWM_KHZ(25), 0);
    (void)pwm_set_dt(&pwm_inc, PWM_KHZ(25), 0);

    /* ---- Stop the Zephyr-started edge-aligned PWM ---- */
    NRF_PWM0->EVENTS_STOPPED = 0;
    NRF_PWM0->TASKS_STOP     = 1;
    for (int i = 0; i < 1000 && !NRF_PWM0->EVENTS_STOPPED; i++) {
        k_busy_wait(1);
    }
    NRF_PWM0->EVENTS_STOPPED = 0;

    /* ---- Init sequence buffer at 50% duty (neutral, zero torque) ---- */
    m_seq_buf[0] = duty_to_compare(0.5f);
    m_seq_buf[1] = duty_to_compare(0.5f);
    m_seq_buf[2] = duty_to_compare(0.5f);
    m_seq_buf[3] = 0x8000u; /* unused channel: 0% duty, inverted */

    /* ---- Configure NRF_PWM0 for CENTER-ALIGNED (UP_AND_DOWN) at 25 kHz ----
     * COUNTERTOP=320, 16 MHz clock -> period = 2*320/16e6 = 40 us = 25 kHz.
     * INDIVIDUAL decoder: each of the 4 entries in m_seq_buf sets one channel's
     * compare value independently (INA/INB/INC/unused).
     * The SEQEND0->SEQSTART0 shortcut creates an infinite hardware loop without
     * any CPU intervention; write_duty_cycles() only updates m_seq_buf[]. */
    NRF_PWM0->PRESCALER  = PWM_PRESCALER_PRESCALER_DIV_1; /* 16 MHz, no div */
    NRF_PWM0->MODE       = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos);
    NRF_PWM0->COUNTERTOP = PWM_CENTER_ALIGNED_COUNTERTOP;
    NRF_PWM0->DECODER    = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                           (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM0->LOOP       = 1; /* 1 repetition; LOOPSDONE->SEQSTART0 shortcut makes it infinite */

    NRF_PWM0->SEQ[0].PTR      = (uint32_t)(uintptr_t)m_seq_buf;
    NRF_PWM0->SEQ[0].CNT      = PWM_SEQ_SIZE;
    NRF_PWM0->SEQ[0].REFRESH  = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    /* Auto-restart: after LOOP=1 period LOOPSDONE fires and SEQSTART[0] is
     * triggered again by shortcut -> infinite hardware loop, CPU-free.
     * The DMA re-reads m_seq_buf[] each iteration. */
    NRF_PWM0->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;

    NRF_PWM0->TASKS_SEQSTART[0] = 1;
    LOG_INF("NRF_PWM0 center-aligned (UP_AND_DOWN) started: COUNTERTOP=%u (25 kHz)",
            PWM_CENTER_ALIGNED_COUNTERTOP);

    /* Compute and cache the effective period in ns for callers. */
    long freq = (driver->pwm_frequency > 0) ? driver->pwm_frequency : 25000;
    g_period_ns = (uint32_t)(1000000000UL / (unsigned long)freq);

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
    LOG_INF("DRV8311H 3PWM driver ready (CENTER-ALIGNED 25 kHz): freq=%ld Hz, period=%u ns, Vsupply=%.1f V",
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

void bldc_driver_3pwm_sleep(bldc_driver_3pwm_t *driver)
{
    /* Zero the bridge first so no phase is left energised, then drive nSLEEP
     * LOW to put the DRV8311H into its low-power sleep state. */
    if (driver != NULL && driver->initialized) {
        write_duty_cycles(g_period_ns, 0.0f, 0.0f, 0.0f);
        driver->dc_a = 0.0f;
        driver->dc_b = 0.0f;
        driver->dc_c = 0.0f;
    }
    if (gpio_is_ready_dt(&gpio_sleep)) {
        gpio_pin_set_dt(&gpio_sleep, 0);   /* nSLEEP LOW = sleep */
        LOG_INF("DRV8311H nSLEEP driven LOW (sleep)");
    }
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
