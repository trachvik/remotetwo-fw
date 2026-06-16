#include "current_sense.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <math.h>

#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_pwm.h>
#include <hal/nrf_saadc.h>

LOG_MODULE_REGISTER(current_sense, LOG_LEVEL_DBG);

/*
 * PWM-synchronised low-side current sensing for the DRV8311H.
 *
 * Why this exists: the DRV8311H measures phase current across the LOW-SIDE
 * shunt, so a phase current is only visible on SOx while that phase's low-side
 * FET conducts. Sampling the SAADC asynchronously mostly catches the high-side
 * window (or the amplifier mid-rail bias) and reads ~0 A. Worse, when all three
 * channels are sampled at the same bias the Clarke common-mode subtraction
 * (mid = (a+b+c)/3) makes Iq = Id = exactly 0.000, which winds up the current
 * PID against dead feedback.
 *
 * Fix: take the SAADC over with nrfx in advanced mode and hardware-trigger
 * NRF_SAADC_TASK_SAMPLE from NRF_PWM0's PWMPERIODEND event via GPPI, so every
 * conversion is phase-locked to the PWM period. Combined with inverted motor
 * PWM polarity (see bldc_driver_3pwm.c) the low-side conduction window sits at
 * the very start of each period, right where PWMPERIODEND fires.
 *
 * Channel / pin map (scan order follows channel_index ascending):
 *   index 0 -> AIN6 = P0.27 = SOA
 *   index 1 -> AIN5 = P0.26 = SOB
 *   index 2 -> AIN4 = P0.25 = SOC
 */

/* Clarke transform constants (SimpleFOC: _1_SQRT3, _2_SQRT3). */
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

/* DRV8311H CSA gain: 2 V/A. Phase current [A] = Vshunt / 2.0. */
#define CSA_GAIN_V_PER_A 2.0f
#define CSA_AMPS_PER_V   (1.0f / CSA_GAIN_V_PER_A)

#define NUM_CH        3
#define SCANS_PER_BUF 8
#define BUF_LEN       (NUM_CH * SCANS_PER_BUF)

/* Internal 0.6 V reference with gain 1/6 -> full-scale 3.6 V single-ended,
 * which spans the 0..3.3 V CSA output (biased near AVDD/2) with headroom.
 * 12-bit resolution => 4096 codes over 3.6 V. */
#define SAADC_FS_VOLTS      3.6f
#define SAADC_FULL_SCALE    4096.0f
#define SAADC_RAW_TO_VOLTS(raw) (((float)(raw)) * SAADC_FS_VOLTS / SAADC_FULL_SCALE)

#define SAADC_IRQ_PRIO 6

/* Double buffer for continuous DMA-style conversion. */
static nrf_saadc_value_t m_buf[2][BUF_LEN];
static volatile uint8_t  m_next_buf = 0;

/* Latest PWM-synced scan, [0]=SOA [1]=SOB [2]=SOC (raw 12-bit codes). */
static volatile int16_t  g_raw[NUM_CH] = {0, 0, 0};
static volatile uint32_t g_scan_count = 0;

static bool  g_ready = false;
static float g_offset_v[NUM_CH] = {0.0f, 0.0f, 0.0f};
static float g_gain_sign = 1.0f;

/* GPPI connection handles (multi-domain API). */
static nrfx_gppi_handle_t h_sample;   /* PWMPERIODEND -> SAADC SAMPLE */
static nrfx_gppi_handle_t h_start;    /* SAADC END    -> SAADC START  */
static volatile bool      g_armed = false;

static nrfx_saadc_channel_t m_channels[NUM_CH] = {
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN6, 0), /* SOA */
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN5, 1), /* SOB */
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN4, 2), /* SOC */
};

/* ------------------------------------------------------------------ */
/* SAADC event handler (runs in SAADC IRQ context)                     */
/* ------------------------------------------------------------------ */
static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
    switch (p_event->type) {
    case NRFX_SAADC_EVT_CALIBRATEDONE:
        /* Offset calibration finished -> kick the first START. */
        (void)nrfx_saadc_mode_trigger();
        break;

    case NRFX_SAADC_EVT_READY:
        /* Peripheral has acquired the first buffer; from now on every
         * PWMPERIODEND triggers a scan. Enable the sampling GPPI link. */
        nrfx_gppi_conn_enable(h_sample);
        g_armed = true;
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
        /* Supply the alternate buffer to keep conversion continuous. */
        m_next_buf ^= 1;
        (void)nrfx_saadc_buffer_set(m_buf[m_next_buf], BUF_LEN);
        break;

    case NRFX_SAADC_EVT_DONE: {
        nrf_saadc_value_t *b = p_event->data.done.p_buffer;
        uint16_t n = p_event->data.done.size;
        if (n >= NUM_CH) {
            /* Most recent scan = last NUM_CH samples of the filled buffer. */
            g_raw[0] = b[n - 3];
            g_raw[1] = b[n - 2];
            g_raw[2] = b[n - 1];
            g_scan_count++;
        }
        break;
    }

    default:
        break;
    }
}

/* Non-blocking snapshot of the latest synced scan, converted to volts. */
static void isense_read_volts(float v_out[NUM_CH])
{
    int16_t raw[NUM_CH];
    unsigned int key = irq_lock();
    raw[0] = g_raw[0];
    raw[1] = g_raw[1];
    raw[2] = g_raw[2];
    irq_unlock(key);

    for (int i = 0; i < NUM_CH; i++) {
        /* Clamp negatives (single-ended can read slightly below 0). */
        int16_t r = (raw[i] < 0) ? 0 : raw[i];
        v_out[i] = SAADC_RAW_TO_VOLTS(r);
    }
}

int current_sense_init(void)
{
    int ret;

    g_ready = false;
    g_armed = false;
    g_scan_count = 0;
    m_next_buf = 0;

    /* Connect and enable the SAADC IRQ ourselves (CONFIG_ADC is off, so the
     * Zephyr ADC driver does not own the peripheral). */
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), SAADC_IRQ_PRIO,
                nrfx_saadc_irq_handler, NULL, 0);

    ret = nrfx_saadc_init(SAADC_IRQ_PRIO);
    if (ret != 0 && ret != -EALREADY) {
        LOG_ERR("nrfx_saadc_init failed (%d) - current sense UNAVAILABLE", ret);
        return ret;
    }
    irq_enable(NRFX_IRQ_NUMBER_GET(NRF_SAADC));

    /* Override the macro defaults: gain 1/6 (full scale 3.6 V, NOT 0.6 V) and
     * a short 3 us acquisition so all three channels fit inside the low-side
     * conduction window. */
    for (int i = 0; i < NUM_CH; i++) {
        m_channels[i].channel_config.gain     = NRF_SAADC_GAIN1_6;
        m_channels[i].channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
    }

    ret = nrfx_saadc_channels_config(m_channels, NUM_CH);
    if (ret != 0) {
        LOG_ERR("nrfx_saadc_channels_config failed (%d)", ret);
        return ret;
    }

    uint32_t ch_mask = nrfx_saadc_channels_configured_get();

    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    /* internal_timer_cc = 0 and start_on_end = false already: we drive both
     * SAMPLE and START from hardware (PWM + GPPI). */

    ret = nrfx_saadc_advanced_mode_set(ch_mask, NRF_SAADC_RESOLUTION_12BIT,
                                       &adv, saadc_handler);
    if (ret != 0) {
        LOG_ERR("nrfx_saadc_advanced_mode_set failed (%d)", ret);
        return ret;
    }

    ret = nrfx_saadc_buffer_set(m_buf[0], BUF_LEN);
    if (ret != 0) {
        LOG_ERR("nrfx_saadc_buffer_set failed (%d)", ret);
        return ret;
    }

    /* GPPI link #1: PWM0 PWMPERIODEND -> SAADC SAMPLE (the phase lock). */
    uint32_t pwm_evt     = nrf_pwm_event_address_get(NRF_PWM0, NRF_PWM_EVENT_PWMPERIODEND);
    uint32_t saadc_smpl  = nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
    ret = nrfx_gppi_conn_alloc(pwm_evt, saadc_smpl, &h_sample);
    if (ret != 0) {
        LOG_ERR("GPPI sample conn alloc failed (%d)", ret);
        return ret;
    }

    /* GPPI link #2: SAADC END -> SAADC START (auto re-arm next buffer). */
    uint32_t saadc_end   = nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END);
    uint32_t saadc_start = nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START);
    ret = nrfx_gppi_conn_alloc(saadc_end, saadc_start, &h_start);
    if (ret != 0) {
        LOG_ERR("GPPI start conn alloc failed (%d)", ret);
        return ret;
    }
    nrfx_gppi_conn_enable(h_start);

    /* Run the SAADC's internal offset calibration, then start. If calibration
     * is unsupported, start sampling directly. */
    ret = nrfx_saadc_offset_calibrate(saadc_handler);
    if (ret != 0) {
        LOG_WRN("SAADC offset calibrate unavailable (%d), starting directly", ret);
        (void)nrfx_saadc_mode_trigger();
    }

    /* Wait for the peripheral to arm (READY event). */
    for (int i = 0; i < 200 && !g_armed; i++) {
        k_msleep(1);
    }
    if (!g_armed) {
        LOG_ERR("SAADC did not arm - current sense UNAVAILABLE");
        return -ETIMEDOUT;
    }

    /* Kick a few software SAMPLE triggers so the probe read below works even
     * if the motor PWM is momentarily idle. Once the motor runs, PWMPERIODEND
     * keeps the scans flowing on its own. */
    for (int i = 0; i < BUF_LEN + NUM_CH; i++) {
        nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
        k_busy_wait(40);
    }
    k_msleep(2);

    g_ready = true;

    float v[NUM_CH];
    isense_read_volts(v);
    LOG_INF("SAADC PWM-sync ready: probe SOA=%.3f SOB=%.3f SOC=%.3f V (scans=%u)",
            (double)v[0], (double)v[1], (double)v[2], g_scan_count);
    return 0;
}

bool current_sense_is_ready(void)
{
    return g_ready;
}

int current_sense_calibrate_offsets(void)
{
    if (!g_ready) {
        return -ENODEV;
    }

    const int rounds = 500;
    double acc[NUM_CH] = {0.0, 0.0, 0.0};
    uint32_t start_scans = g_scan_count;

    for (int n = 0; n < rounds; n++) {
        float v[NUM_CH];
        isense_read_volts(v);
        acc[0] += v[0];
        acc[1] += v[1];
        acc[2] += v[2];
        k_msleep(1);
    }

    for (int i = 0; i < NUM_CH; i++) {
        g_offset_v[i] = (float)(acc[i] / rounds);
    }

    uint32_t scans = g_scan_count - start_scans;
    LOG_INF("offsets calibrated: SOA=%.3f SOB=%.3f SOC=%.3f V (expect ~AVDD/2, %u synced scans)",
            (double)g_offset_v[0], (double)g_offset_v[1], (double)g_offset_v[2], scans);
    if (scans == 0) {
        LOG_WRN("NO PWM-synced scans during calibration - is PWM0 running?");
    }
    return 0;
}

phase_current_t current_sense_get_phase_currents(void)
{
    phase_current_t c = {0.0f, 0.0f, 0.0f};

    if (!g_ready) {
        return c;
    }

    float v[NUM_CH];
    isense_read_volts(v);

    c.a = (v[0] - g_offset_v[0]) * CSA_AMPS_PER_V * g_gain_sign;
    c.b = (v[1] - g_offset_v[1]) * CSA_AMPS_PER_V * g_gain_sign;
    c.c = (v[2] - g_offset_v[2]) * CSA_AMPS_PER_V * g_gain_sign;
    return c;
}

void current_sense_get_foc_currents(float angle_el, float *id, float *iq)
{
    phase_current_t c = current_sense_get_phase_currents();

    /* Clarke transform with all three phases measured (SimpleFOC getABCurrents):
     * subtract the common-mode mid = (a+b+c)/3 (uses a+b+c=0 to reject noise),
     * then i_alpha = a, i_beta = (a + 2b)/sqrt(3). */
    float mid = (1.0f / 3.0f) * (c.a + c.b + c.c);
    float a = c.a - mid;
    float b = c.b - mid;
    float i_alpha = a;
    float i_beta  = _1_SQRT3 * a + _2_SQRT3 * b;

    /* Park transform (SimpleFOC getFOCCurrents). */
    float ct = cosf(angle_el);
    float st = sinf(angle_el);
    if (id) {
        *id = i_alpha * ct + i_beta * st;
    }
    if (iq) {
        *iq = i_beta * ct - i_alpha * st;
    }
}

void current_sense_set_gain_sign(float sign)
{
    g_gain_sign = (sign < 0.0f) ? -1.0f : 1.0f;
}

float current_sense_get_gain_sign(void)
{
    return g_gain_sign;
}

void current_sense_get_offsets(float *oa, float *ob, float *oc)
{
    if (oa) *oa = g_offset_v[0];
    if (ob) *ob = g_offset_v[1];
    if (oc) *oc = g_offset_v[2];
}

void current_sense_get_raw_volts(float *va, float *vb, float *vc)
{
    float v[NUM_CH];
    isense_read_volts(v);
    if (va) *va = v[0];
    if (vb) *vb = v[1];
    if (vc) *vc = v[2];
}

uint32_t current_sense_get_scan_count(void)
{
    return g_scan_count;
}
