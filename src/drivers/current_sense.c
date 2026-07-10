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
#include <hal/nrf_timer.h>

LOG_MODULE_REGISTER(current_sense, LOG_LEVEL_DBG);

/*
 * ============================================================================
 * Hardware-timed low-side current sensing for the DRV8311H.
 * ============================================================================
 *
 * Expected hardware flow (all CPU-less, jitter-free):
 *
 *   PWM0.PWMPERIODEND --(DPPI ch A)--> TIMER2.CLEAR     (re-sync each period)
 *   TIMER2.COMPARE[0] --(DPPI ch B)--> SAADC.SAMPLE     (sample at the chosen
 *                                                        point in the period)
 *   SAADC.END         --(DPPI ch C)--> SAADC.START      (auto re-arm EasyDMA)
 *
 *   SAADC EasyDMA writes the three conversions to RAM and raises an IRQ only
 *   on buffer DONE - the CPU never touches a SAMPLE trigger.
 *
 * TIMER2 is a free-running 16 MHz counter that is hardware-CLEARED at the start
 * of every PWM period (PWMPERIODEND fires at the period boundary). Its
 * COMPARE[0] therefore fires at a fixed, jitter-free offset into every PWM
 * period. The offset is ISENSE_SAMPLE_FRACTION of the period.
 *
 * -------------------------- ALIGNMENT CAVEAT --------------------------------
 * The motor PWM (bldc_driver_3pwm.c) uses the Zephyr nrfx PWM driver, which is
 * EDGE-ALIGNED (count-up) with NORMAL polarity: every phase's high-side pulse
 * starts at count 0. The low-side FET of a phase conducts during the REMAINDER
 * of the period, so the window in which ALL THREE low-side FETs conduct
 * simultaneously sits at the END of the period (just before PWMPERIODEND), NOT
 * at its geometric centre. For 50 %-ish duty the geometric centre is right on
 * the high-side switching edge = worst-case switching noise.
 *
 * Two correct options:
 *   (a) Switch the motor PWM to CENTRE-ALIGNED (up-and-down) mode - then the
 *       period centre genuinely is the middle of the common low-side window and
 *       ISENSE_SAMPLE_FRACTION = 0.5 is ideal. (Requires driving NRF_PWM0 via
 *       nrfx_pwm in NRF_PWM_MODE_UP_AND_DOWN, bypassing the Zephyr PWM driver.)
 *   (b) Keep edge-aligned and move the sample point toward the END of the
 *       period (ISENSE_SAMPLE_FRACTION ~ 0.92).
 * ISENSE_SAMPLE_FRACTION below is the single knob for this.
 *
 * Channel / pin map (SAADC scan order = channel_index ascending):
 *   index 0 -> AIN5 = P0.26 = SOC   (scanned first)
 *   index 1 -> AIN4 = P0.25 = SOB   (scanned second)
 *   index 2 -> AIN6 = P0.27 = SOA   (scanned third; value overridden by Kirchhoff)
 * NOTE: AIN4/AIN5 are swapped relative to DRV8311H SO_B/SO_C pin labels.
 */

/* Fraction of the PWM period at which TIMER2.COMPARE[0] fires the SAADC sample.
 *
 * The nRF PWM with NORMAL polarity (Zephyr default) generates:
 *   HIGH from counter=0 to COMPARE  (high-side ON  -> shunt not in path -> CSA ~0)
 *   LOW  from COMPARE to COUNTERTOP (low-side  ON  -> shunt carries current -> CSA valid)
 * Sampling at 0.1 (counter=64) therefore always lands in the HIGH zone -> false zeros.
 *
 * FIX: bldc_driver_3pwm.c uses PWM_LOWSIDE_SYNC=1 which writes the pulse as
 *   pwm_set(..., period - pa, PWM_POLARITY_INVERTED)
 * With INVERTED polarity the bit-15 polarity flag in the sequence data is cleared,
 * causing the nRF PWM hardware to output:
 *   LOW  from counter=0 to (period-pa)/period * COUNTERTOP  (low-side ON  -> CSA valid)
 *   HIGH from (period-pa)/period * COUNTERTOP to COUNTERTOP (high-side ON)
 * The motor sees identical phase voltages (same high-side conduction time),
 * but the quiet CSA window is now at the START of every period.
 *
 * PWMPERIODEND fires at counter=COUNTERTOP.  TIMER2 clears immediately.
 * After fraction * period COMPARE0 fires -> SAADC SAMPLE.
 *
 * 2-CHANNEL SCAN (SOC → SOB only) - why SOA (AIN6) is excluded:
 * Each SAADC channel takes ~5 us (3 us acq + ~2 us conv = 80 ticks at 16 MHz).
 * Starting at fraction=0.04 (counter=26, 1.6 us after LOW-side turn-on):
 *   ch0  SOC (AIN5): counter  26 - 106   quiet_C > 106 for Vc < 1.25 V      ✓
 *   ch1  SOB (AIN4): counter 106 - 186   quiet_B > 186 for Vb < 0.79 V      ✓
 * (Quiet window = (1-dc)*640; dc = 0.5 + V/Vbatt)
 *
 * If a 3rd channel (SOA/AIN6) were added, its acquisition window (186-266) would
 * overlap the phase-A top-FET switching edge (~counter 200-250 at haptic voltages).
 * The DRV8311H CSA transient on AIN6 at that instant creates a ground/supply bounce
 * that corrupts ch0 and ch1 results for the same scan.  Empirically confirmed:
 * adding ch2 caused discontinuities on ALL three reconstructed phases.
 *
 * Phase A is recovered via Ia = -(Ib + Ic) (Kirchhoff; balanced load).
 * BUF_LEN = 2 * SCANS_PER_BUF is always a clean multiple of 2. */
#define ISENSE_SAMPLE_FRACTION 0.04f

/* Phase-A reconstruction.
 * ON: SOA is not scanned; Ia is recovered via Ia = -(Ib + Ic) (Kirchhoff).
 * SOA (AIN6) is deliberately excluded: a 3rd channel scan would cross the
 * phase-A FET switching transient and corrupt all channels (empirically verified). */
#define ISENSE_RECONSTRUCT_PHASE_A 1

/* TIMER instance used as the PWM-locked sample timer. TIMER2 is not claimed by
 * the Zephyr system clock (RTC1) nor by any driver on this target. */
#define ISENSE_TIMER          NRF_TIMER2
#define ISENSE_TIMER_PRESCALER 0UL          /* 16 MHz base / 2^0 = 16 MHz tick */
#define ISENSE_TIMER_HZ       16000000UL

/* Clarke transform constants (SimpleFOC: _1_SQRT3, _2_SQRT3). */
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

/* DRV8311H CSA: SOx = VREF/2 +/- (GCSA * IOUTx), GCSA = 2 V/A.
 * => IOUTx [A] = (SOx - offset) / GCSA, offset ~ VREF/2 (measured, not assumed). */
#define CSA_GAIN_V_PER_A 2.0f
#define CSA_AMPS_PER_V   (1.0f / CSA_GAIN_V_PER_A)

#define NUM_CH        3                       /* reported phase channels SOA,SOB,SOC */
#define NUM_SCAN_CH   2                       /* SOB + SOC only; SOA reconstructed */
#define SCANS_PER_BUF 20
#define BUF_LEN       (NUM_SCAN_CH * SCANS_PER_BUF)

/* Internal 0.6 V reference with gain 1/6 -> full-scale 3.6 V single-ended,
 * which spans the 0..3.3 V CSA output (biased near VREF/2). 12-bit. */
#define SAADC_FS_VOLTS      3.6f
#define SAADC_FULL_SCALE    4096.0f
#define SAADC_RAW_TO_VOLTS(raw) (((float)(raw)) * SAADC_FS_VOLTS / SAADC_FULL_SCALE)

#define SAADC_IRQ_PRIO 6

/* Double buffer for continuous EasyDMA conversion. */
static nrf_saadc_value_t m_buf[2][BUF_LEN];
static volatile uint8_t  m_next_buf = 0;

/* Latest hardware-synced scan, [0]=SOA [1]=SOB [2]=SOC (raw 12-bit codes). */
static volatile int16_t  g_raw[NUM_CH] = {0, 0, 0};
static volatile uint32_t g_scan_count = 0;

static bool  g_ready = false;
static float g_offset_v[NUM_CH] = {0.0f, 0.0f, 0.0f};
static float g_gain_sign = 1.0f;

/* Cache of the most recent raw phase voltages [V] and corrected currents [A],
 * for diagnostics and the SerialPlot stream (no re-trigger needed). */
static float g_last_v[NUM_CH] = {0.0f, 0.0f, 0.0f};
static float g_last_i[NUM_CH] = {0.0f, 0.0f, 0.0f};

/* GPPI (DPPI on the nRF5340) connection handles. */
static nrfx_gppi_handle_t       h_clear;       /* PWM PWMPERIODEND -> TIMER CLEAR            */
static nrfx_gppi_handle_t       h_sample;      /* TIMER COMPARE0  -> SAADC SAMPLE (gated)   */
static nrfx_gppi_handle_t       h_dis;         /* SAADC END       -> group DIS (gate closed) */
static nrfx_gppi_handle_t       h_en;          /* SAADC STARTED   -> group EN  (gate open)  */
static nrfx_gppi_group_handle_t g_sample_grp;  /* DPPI group controlling h_sample enable     */
static volatile bool      g_armed = false;
static volatile uint32_t  g_buf_req_count = 0;

static nrfx_saadc_channel_t m_channels[NUM_SCAN_CH] = {
    /* IMPORTANT: use NRFX_ANALOG_EXTERNAL_AINx (0-indexed: AIN0=0, AIN5=5, AIN4=4)
     * NOT NRF_SAADC_INPUT_AINx (1-indexed). channel_idx sets the SAADC HW channel
     * register (0..7) and determines scan order (ascending index).
     * Physical mapping (empirically verified): AIN5/P0.26 = SOC, AIN4/P0.25 = SOB,
     * AIN6/P0.27 = SOA (inverted B/C relative to DRV8311H SO_x pin names). */
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRFX_ANALOG_EXTERNAL_AIN5, 0), /* SOC - P0.26, scanned 1st */
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRFX_ANALOG_EXTERNAL_AIN4, 1), /* SOB - P0.25, scanned 2nd */
};

/* ------------------------------------------------------------------ */
/* SAADC event handler (runs in SAADC IRQ context).                    */
/* The ONLY CPU work per measurement: copy the freshest 3 samples out  */
/* of the DMA buffer when EasyDMA signals DONE.                        */
/* ------------------------------------------------------------------ */
static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
    switch (p_event->type) {
    case NRFX_SAADC_EVT_CALIBRATEDONE:
        (void)nrfx_saadc_mode_trigger();
        break;

    case NRFX_SAADC_EVT_READY:
        /* Peripheral armed; from now on TIMER COMPARE0 drives every scan. */
        nrfx_gppi_conn_enable(h_sample);
        g_armed = true;
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
        g_buf_req_count++;
        m_next_buf ^= 1;
        (void)nrfx_saadc_buffer_set(m_buf[m_next_buf], BUF_LEN);
        break;

    case NRFX_SAADC_EVT_DONE: {
        nrf_saadc_value_t *b = p_event->data.done.p_buffer;
        uint16_t n = p_event->data.done.size;
        /* Use the LAST complete scan in the buffer (not an average).
         *
         * Averaging all 100 scans over the 4ms buffer window would
         * smear the signal over 4ms.  At haptic oscillation frequencies
         * (50-200 Hz electrical = 5-18 Hz mechanical × 11 pole pairs),
         * a 4ms average covers 0.2-0.8 electrical cycles and reduces
         * signal amplitude by sinc(π·cycles): at 100 Hz -24%, at 200 Hz
         * -77%, at 220 Hz -96%.  Practically this kills the current
         * feedback needed for FOC and makes SerialPlot show near-zero.
         *
         * Using only the last complete scan gives the INSTANTANEOUS
         * current at the most recent PWM quiet window.
         *
         * B/C swap protection: `(n / NUM_SCAN_CH - 1) * NUM_SCAN_CH`
         * rounds down to the last COMPLETE pair.  Even if n is odd
         * (STOPPED fired early), `last` is always even-aligned so
         * ch0=SOC lands at an even offset and ch1=SOB at odd — no swap. */
        uint16_t scans = n / NUM_SCAN_CH;
        if (scans > 0) {
            uint16_t last = (uint16_t)((scans - 1u) * (uint16_t)NUM_SCAN_CH);
            g_raw[2] = b[last];       /* SOC - ch0, always at even offset */
            g_raw[1] = b[last + 1];   /* SOB - ch1, always at odd offset  */
            g_scan_count++;
        }
        break;
    }

    case NRFX_SAADC_EVT_FINISHED:
        /* The SAADC stopped unexpectedly (secondary buffer was NULL when END
         * fired, e.g. after a spurious STOPPED event).  Re-arm the double
         * buffer and restart so current-sense never goes permanently silent. */
        m_next_buf = 0;
        (void)nrfx_saadc_buffer_set(m_buf[0], BUF_LEN);
        (void)nrfx_saadc_mode_trigger();
        break;

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
        int16_t r = (raw[i] < 0) ? 0 : raw[i];
        v_out[i] = SAADC_RAW_TO_VOLTS(r);
    }

    unsigned int k2 = irq_lock();
    g_last_v[0] = v_out[0];
    g_last_v[1] = v_out[1];
    g_last_v[2] = v_out[2];
    irq_unlock(k2);
}

/* ------------------------------------------------------------------ */
/* TIMER2 + DPPI bridge setup.                                         */
/* ------------------------------------------------------------------ */
static void isense_timer_dppi_setup(void)
{
    /* --- Read the live PWM timing so the sample point tracks the configured
     * PWM frequency instead of a hard-coded value. The Zephyr PWM driver has
     * already programmed COUNTERTOP/PRESCALER by the time we get here. The HAL
     * has no getter for these, so read the registers directly. --- */
    uint32_t top = NRF_PWM0->COUNTERTOP & PWM_COUNTERTOP_COUNTERTOP_Msk;
    uint32_t pre = NRF_PWM0->PRESCALER  & PWM_PRESCALER_PRESCALER_Msk;

    /* PWM clock = 16 MHz >> pre, so one PWM tick = (1 << pre) TIMER (16 MHz)
     * ticks.  In UP_AND_DOWN (center-aligned) mode the full period spans both
     * the upswing and the downswing = 2 * COUNTERTOP ticks; in UP (edge-aligned)
     * mode the period is just COUNTERTOP ticks. */
    uint32_t pwm_mode = NRF_PWM0->MODE & PWM_MODE_UPDOWN_Msk;
    bool center_aligned = (pwm_mode == (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos));
    uint32_t period_ticks = (center_aligned ? 2u : 1u) * (top << pre);
    uint32_t sample_ticks = (uint32_t)((float)period_ticks * ISENSE_SAMPLE_FRACTION);
    if (sample_ticks == 0) {
        sample_ticks = 1;
    }

    /* TIMER2 as a 32-bit, 16 MHz, event-only counter (no interrupts). */
    nrf_timer_mode_set(ISENSE_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(ISENSE_TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_prescaler_set(ISENSE_TIMER, ISENSE_TIMER_PRESCALER);
    nrf_timer_cc_set(ISENSE_TIMER, NRF_TIMER_CC_CHANNEL0, sample_ticks);
    nrf_timer_event_clear(ISENSE_TIMER, NRF_TIMER_EVENT_COMPARE0);
    nrf_timer_task_trigger(ISENSE_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(ISENSE_TIMER, NRF_TIMER_TASK_START);

    LOG_INF("isense timer: PWM top=%u pre=%u %s -> period=%u ticks, sample@%u ticks (%.0f%%)",
            top, pre, center_aligned ? "UP_AND_DOWN" : "UP",
            period_ticks, sample_ticks,
            (double)(ISENSE_SAMPLE_FRACTION * 100.0f));
}

int current_sense_init(void)
{
    int ret;

    g_ready = false;
    g_armed = false;
    g_scan_count = 0;
    m_next_buf = 0;
    g_gain_sign = 1.0f;
    for (int i = 0; i < NUM_CH; i++) {
        g_offset_v[i] = 0.0f;
        g_last_v[i]   = 0.0f;
        g_last_i[i]   = 0.0f;
    }

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

    /* Override macro defaults: gain 1/6 (full scale 3.6 V) and minimum 3 us
     * acquisition time for all NUM_SCAN_CH=2 channels. */
    for (int i = 0; i < NUM_SCAN_CH; i++) {
        m_channels[i].channel_config.gain     = NRF_SAADC_GAIN1_6;
        m_channels[i].channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
    }

    ret = nrfx_saadc_channels_config(m_channels, NUM_SCAN_CH);
    if (ret != 0) {
        LOG_ERR("nrfx_saadc_channels_config failed (%d)", ret);
        return ret;
    }

    uint32_t ch_mask = nrfx_saadc_channels_configured_get();

    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    /* start_on_end=true: nrfx writes RESULT.PTR then fires START as an atomic pair
     * inside saadc_event_end_handle via nrfy_saadc_buffer_latch(p_reg, false).
     * This guarantees the correct buffer address is latched at START.
     * The race window (SAADC stopped between END and START) is eliminated by
     * the DPPI group gating on h_sample below — not by deferring START. */
    adv.start_on_end = true;

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

    /* Bring up TIMER2 + program its centre-of-period compare. */
    isense_timer_dppi_setup();

    /* DPPI ch A: PWM0 PWMPERIODEND -> TIMER2 CLEAR
     * PWMPERIODEND fires every 40 us (25 kHz) when the counter reaches COUNTERTOP.
     * This hardware-locked event resets TIMER2.  COMPARE0 fires 0.1*period = 4 us
     * later at counter=64, which is inside the low-side quiet window for all
     * typical haptic duty cycles (dc < 0.9 with PWM_LOWSIDE_SYNC=1). */
    uint32_t pwm_evt    = nrf_pwm_event_address_get(NRF_PWM0, NRF_PWM_EVENT_PWMPERIODEND);
    uint32_t tmr_clear  = nrf_timer_task_address_get(ISENSE_TIMER, NRF_TIMER_TASK_CLEAR);
    ret = nrfx_gppi_conn_alloc(pwm_evt, tmr_clear, &h_clear);
    if (ret != 0) {
        LOG_ERR("GPPI clear conn alloc failed (%d)", ret);
        return ret;
    }
    nrfx_gppi_conn_enable(h_clear);

    /* DPPI ch B: TIMER2 COMPARE0 -> SAADC SAMPLE (the actual phase-locked
     * sample point). Enabled only once the SAADC reports READY. */
    uint32_t tmr_cmp    = nrf_timer_event_address_get(ISENSE_TIMER, NRF_TIMER_EVENT_COMPARE0);
    uint32_t saadc_smpl = nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
    ret = nrfx_gppi_conn_alloc(tmr_cmp, saadc_smpl, &h_sample);
    if (ret != 0) {
        LOG_ERR("GPPI sample conn alloc failed (%d)", ret);
        return ret;
    }

    /* DPPI group gating for h_sample: prevents SAMPLE from reaching a stopped SAADC.
     *
     * Root cause of B/C channel swap: between hardware EVENTS_END and the software
     * START issued by nrfy_saadc_buffer_latch() (~10-20 µs IRQ latency, potentially
     * 50-100 µs with BLE), the SAADC is stopped.  Any TIMER COMPARE0 → SAADC SAMPLE
     * trigger arriving in this window shifts the internal channel MUX by one, swapping
     * ch0 (SOC) and ch1 (SOB) for the ENTIRE next buffer.
     *
     * Fix: gate h_sample with a DPPI group.
     *   EVENTS_END    → h_dis → group DIS → CHEN[h_sample] = 0  (hardware, 0 cycles)
     *   EVENTS_STARTED → h_en → group EN  → CHEN[h_sample] = 1  (hardware, 0 cycles)
     * STARTED fires immediately after START (issued by nrfy_saadc_buffer_latch), so
     * SAMPLE is re-enabled the instant the SAADC is running again. */
    {
        uint32_t domain_id = nrfx_gppi_domain_id_get(
            nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END));
        ret = nrfx_gppi_group_alloc(domain_id, &g_sample_grp);
        if (ret != 0) {
            LOG_ERR("GPPI group alloc failed (%d)", ret);
            return ret;
        }
        nrfx_gppi_group_ch_add(g_sample_grp, (uint32_t)h_sample);

        /* h_dis: SAADC END -> group DIS (close gate at END, hardware, 0 latency) */
        uint32_t saadc_end_evt     = nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END);
        uint32_t grp_dis_task      = nrfx_gppi_group_task_dis_addr(g_sample_grp);
        ret = nrfx_gppi_conn_alloc(saadc_end_evt, grp_dis_task, &h_dis);
        if (ret != 0) {
            LOG_ERR("GPPI h_dis alloc failed (%d)", ret);
            return ret;
        }
        nrfx_gppi_conn_enable(h_dis);

        /* h_en: SAADC STARTED -> group EN (re-open gate when SAADC is running again) */
        uint32_t saadc_started_evt = nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED);
        uint32_t grp_en_task       = nrfx_gppi_group_task_en_addr(g_sample_grp);
        ret = nrfx_gppi_conn_alloc(saadc_started_evt, grp_en_task, &h_en);
        if (ret != 0) {
            LOG_ERR("GPPI h_en alloc failed (%d)", ret);
            return ret;
        }
        nrfx_gppi_conn_enable(h_en);
    }

    /* ---- DPPI diagnostic dump ---- */
    {
        uint32_t pub_pwm  = *((volatile uint32_t *)(nrf_pwm_event_address_get(NRF_PWM0, NRF_PWM_EVENT_PWMPERIODEND) + 0x80U));
        uint32_t sub_tmr  = *((volatile uint32_t *)(nrf_timer_task_address_get(ISENSE_TIMER, NRF_TIMER_TASK_CLEAR) + 0x80U));
        uint32_t pub_tmr  = *((volatile uint32_t *)(nrf_timer_event_address_get(ISENSE_TIMER, NRF_TIMER_EVENT_COMPARE0) + 0x80U));
        uint32_t sub_saad = *((volatile uint32_t *)(nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE) + 0x80U));
        LOG_INF("DPPI alloc: h_clear=%d h_sample=%d h_dis=%d h_en=%d grp=%d CHEN=0x%08x",
                (int)h_clear, (int)h_sample, (int)h_dis, (int)h_en,
                (int)g_sample_grp, NRF_DPPIC->CHEN);
        LOG_INF("DPPI regs: PUB_PWMPERIODEND=0x%08x SUB_TMRCLEAR=0x%08x",
                pub_pwm, sub_tmr);
        LOG_INF("DPPI regs: PUB_TMRCMP0=0x%08x SUB_SAADCSMP=0x%08x",
                pub_tmr, sub_saad);
        /* Verify TIMER2 is being cleared by PWMPERIODEND via h_clear.
         * Two captures 100 us apart: if h_clear works both reads are <640 (one
         * 25 kHz period = 640 ticks at 16 MHz); if broken counter grows freely. */
        nrf_timer_task_trigger(ISENSE_TIMER, NRF_TIMER_TASK_CAPTURE2);
        uint32_t tc0 = nrf_timer_cc_get(ISENSE_TIMER, NRF_TIMER_CC_CHANNEL2);
        k_busy_wait(100U);
        nrf_timer_task_trigger(ISENSE_TIMER, NRF_TIMER_TASK_CAPTURE2);
        uint32_t tc1 = nrf_timer_cc_get(ISENSE_TIMER, NRF_TIMER_CC_CHANNEL2);
        LOG_INF("TIMER2 cnt: t0=%u t1=%u diff=%d (h_clear OK if diff<640)",
                tc0, tc1, (int)(tc1 - tc0));

        /* Check that PWMPERIODEND actually fires: clear, wait 150 us (4 periods),
         * then read back. fired=1 confirms the DPPI sync source is working. */
        uint32_t pwm_evt_addr = nrf_pwm_event_address_get(NRF_PWM0, NRF_PWM_EVENT_PWMPERIODEND);
        nrf_pwm_event_clear(NRF_PWM0, NRF_PWM_EVENT_PWMPERIODEND);
        k_busy_wait(150U);
        uint32_t pwm_evt_fired = *((volatile uint32_t *)pwm_evt_addr);
        LOG_INF("PWMPERIODEND: addr=0x%08x fired=%u (1=sync OK; 0=PWM stopped/not started)",
                pwm_evt_addr, pwm_evt_fired);
        LOG_INF("Periph addrs: NRF_PWM0=%p NRF_TIMER2=%p NRF_DPPIC=%p",
                (void *)NRF_PWM0, (void *)ISENSE_TIMER, (void *)NRF_DPPIC);
    }

    /* Internal offset calibration, then arm. */
    ret = nrfx_saadc_offset_calibrate(saadc_handler);
    if (ret != 0) {
        LOG_WRN("SAADC offset calibrate unavailable (%d), starting directly", ret);
        (void)nrfx_saadc_mode_trigger();
    }

    for (int i = 0; i < 200 && !g_armed; i++) {
        k_msleep(1);
    }
    if (!g_armed) {
        LOG_ERR("SAADC did not arm - current sense UNAVAILABLE");
        return -ETIMEDOUT;
    }

    /* Kick a few software SAMPLE triggers so the probe read works even if the
     * motor PWM is momentarily idle. Once the motor runs, TIMER2 COMPARE0
     * keeps the scans flowing on its own. */
    for (int i = 0; i < BUF_LEN + NUM_CH; i++) {
        nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
        k_busy_wait(40);
    }
    k_msleep(2);

    g_ready = true;

    float v[NUM_CH];
    isense_read_volts(v);
    LOG_INF("SAADC HW-timed ready: probe SOA=%.3f SOB=%.3f SOC=%.3f V (scans=%u)",
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
        /* SW fallback: guarantee a SAMPLE trigger even when DPPI h_sample is
         * not firing. Each trigger = 1 scan; after 8 scans the buffer fills
         * and g_raw is updated via the DONE callback. */
        /* Phase dither: k_msleep(1) == 1 ms == exactly 25 x 40 µs, so
         * successive SW triggers would all land at the same PWM phase,
         * causing the offset average to lock onto a switching transient
         * instead of the quiet bias level. Adding 1 µs per iteration shifts
         * the trigger phase by 1 µs each round; after 40 rounds we have
         * sampled the full 40 µs period. Over 500 rounds this gives ~12.5
         * complete rotations through the period - a good uniform average. */
        k_busy_wait((uint32_t)(n % 40) + 1U);
        if (g_armed) {
            nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
        }
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
    LOG_INF("offsets calibrated: SOA=%.3f SOB=%.3f SOC=%.3f V (expect ~VREF/2, %u synced scans)",
            (double)g_offset_v[0], (double)g_offset_v[1], (double)g_offset_v[2], scans);
    if (scans == 0) {
        LOG_WRN("NO HW-synced scans during calibration - is PWM0 running? "
                "(buf_req=%u)", g_buf_req_count);
    }
    return 0;
}

phase_current_t current_sense_get_phase_currents(void)
{
    phase_current_t c = {0.0f, 0.0f, 0.0f};

    if (!g_ready) {
        return c;
    }

    /* No SW SAMPLE here. PWMPERIODEND drives TIMER2 at 25 kHz; the DPPI
     * chain fires SAADC SAMPLE every 40 us at a phase-locked counter=64.
     * A 1 kHz SW trigger at a random PWM phase would land in the HIGH zone
     * ~50-90% of the time (counter 0..compare), injecting near-zero readings
     * and causing the false-zero drops seen in earlier firmware versions. */

    float v[NUM_CH];
    isense_read_volts(v);

    /* IOUTx = (SOx - offset) / GCSA, with the alignment sign. */
    float ia_s = (v[0] - g_offset_v[0]) * CSA_AMPS_PER_V * g_gain_sign;
    float ib_s = (v[1] - g_offset_v[1]) * CSA_AMPS_PER_V * g_gain_sign;
    float ic_s = (v[2] - g_offset_v[2]) * CSA_AMPS_PER_V * g_gain_sign;

#if ISENSE_RECONSTRUCT_PHASE_A
    /* The SOA channel (P0.27/AIN6) is hardware-unreliable on this board (reads
     * ~0 regardless of real current - likely a cold joint / damaged shunt or
     * CSA). SOB and SOC are trustworthy, so phase A is recovered exactly from
     * Kirchhoff's current law Ia + Ib + Ic = 0 => Ia = -(Ib + Ic).
     *
     * Only the good SOB/SOC shunts feed the offset-correction matrix from datasheet, and the
     * matrix's tiny A-cross-terms are DROPPED so a bad SOA reading can never
     * leak into B/C. The measured SOA value (ia_s) is therefore unused here. */
    (void)ia_s;
    c.b = 1.000665f * ib_s - 0.019126f * ic_s;
    c.c = 0.001595f * ib_s + 0.998166f * ic_s;
    c.a = -(c.b + c.c);
#else
    /* DRV8311H CSA cross-channel offset correction (datasheet 8.3.10.2). The
     * sense-vs-output FET ground difference induces a small current-dependent
     * offset that distorts multi-shunt FOC; this matrix corrects it. */
    c.a = 1.001152f * ia_s - 0.003375f * ib_s - 0.003103f * ic_s;
    c.b = 0.002369f * ia_s + 1.000665f * ib_s - 0.019126f * ic_s;
    c.c = 0.001234f * ia_s + 0.001595f * ib_s + 0.998166f * ic_s;
#endif

    unsigned int key = irq_lock();
    g_last_i[0] = c.a;
    g_last_i[1] = c.b;
    g_last_i[2] = c.c;
    irq_unlock(key);
    return c;
}

void current_sense_get_foc_currents(float angle_el, float *id, float *iq)
{
    phase_current_t c = current_sense_get_phase_currents();

    /* Clarke transform with all three phases measured (SimpleFOC getABCurrents):
     * subtract the common-mode mid = (a+b+c)/3, then i_alpha = a,
     * i_beta = (a + 2b)/sqrt(3). */
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
    unsigned int key = irq_lock();
    float a = g_last_v[0];
    float b = g_last_v[1];
    float c = g_last_v[2];
    irq_unlock(key);

    if (va) *va = a;
    if (vb) *vb = b;
    if (vc) *vc = c;
}

void current_sense_get_last_currents(float *ia, float *ib, float *ic)
{
    unsigned int key = irq_lock();
    float a = g_last_i[0];
    float b = g_last_i[1];
    float c = g_last_i[2];
    irq_unlock(key);

    if (ia) *ia = a;
    if (ib) *ib = b;
    if (ic) *ic = c;
}

uint32_t current_sense_get_scan_count(void)
{
    return g_scan_count;
}
