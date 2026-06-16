#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Inline current sense for the DRV8311H current-sense amplifiers (CSA),
 * ported 1:1 from SimpleFOC's InlineCurrentSense (C++ -> C).
 *
 *   SOA = P0.27 = SAADC AIN6 (channel 0)
 *   SOB = P0.26 = SAADC AIN5 (channel 1)
 *   SOC = P0.25 = SAADC AIN4 (channel 2)
 *
 * CSA gain = 2 V/A (bidirectional). Phase current [A] = (Vsox - offset) / 2.0,
 * times a polarity sign found by current_sense_align().
 *
 * Design note (the whole reason this module exists): SimpleFOC silently falls
 * back to voltage control when current sense init fails, which hides a broken
 * current loop. This module NEVER hides failure - current_sense_init() returns
 * a negative error code and current_sense_is_ready() stays false, and the motor
 * layer logs a loud, repeated warning instead of pretending it is in current
 * control.
 */

typedef struct {
    float a;
    float b;
    float c;
} phase_current_t;

/* Initialise the SAADC channels. Returns 0 on success, negative errno on
 * failure. Does NOT silently succeed. */
int current_sense_init(void);

/* True only after a successful init (and not after a failure). */
bool current_sense_is_ready(void);

/* Average many samples while the motor produces zero current to capture the
 * amplifier mid-rail offset (~AVDD/2). Must be called with the phases at rest.
 * Returns 0 on success. */
int current_sense_calibrate_offsets(void);

/* Read the three phase currents [A] (offset removed, gain + sign applied). */
phase_current_t current_sense_get_phase_currents(void);

/* Clarke + Park: produce the FOC d/q currents [A] at the given electrical
 * angle. Mirrors SimpleFOC CurrentSense::getFOCCurrents(). */
void current_sense_get_foc_currents(float angle_el, float *id, float *iq);

/* Polarity sign (+1 / -1) applied to every phase current, set by alignment. */
void current_sense_set_gain_sign(float sign);
float current_sense_get_gain_sign(void);

/* Diagnostics for proof logging: the measured zero-current offsets [V]. */
void current_sense_get_offsets(float *oa, float *ob, float *oc);

/* Diagnostics: the most recent PWM-synced raw phase voltages [V] (before offset
 * subtraction) and the running count of synced scans. Used to verify that the
 * SAADC really sees the low-side current window. */
void     current_sense_get_raw_volts(float *va, float *vb, float *vc);
uint32_t current_sense_get_scan_count(void);

#endif /* CURRENT_SENSE_H */
