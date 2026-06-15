#ifndef BLDC_DRIVER_3PWM_H
#define BLDC_DRIVER_3PWM_H

#include <stdint.h>
#include <stdbool.h>

/* NOT_SET constant */
#ifndef NOT_SET
#define NOT_SET -12345.0f
#endif

/* Default power supply voltage */
#define DEF_POWER_SUPPLY 12.0f

/* Driver initialization return codes */
#define DRIVER_INIT_OK     0
#define DRIVER_INIT_FAILED -1

/**
 * DRV8311H 3PWM driver structure.
 *
 * In 3PWM mode the DRV8311H accepts one PWM signal per phase (INA/INB/INC).
 * Complementary low-side switching and dead-time insertion are handled
 * internally by the chip.  INL is the global low-side enable; drive it HIGH
 * permanently.  nSLEEP is driven HIGH at init and never cleared.
 */
typedef struct {
    /* Configuration */
    float voltage_power_supply; /* power supply voltage [V] */
    float voltage_limit;        /* maximum allowed output voltage [V] */
    long  pwm_frequency;        /* PWM frequency [Hz] */

    /* State */
    float dc_a;  /* last applied duty cycle phase A [0,1] */
    float dc_b;  /* last applied duty cycle phase B [0,1] */
    float dc_c;  /* last applied duty cycle phase C [0,1] */

    bool initialized; /* true after successful bldc_driver_3pwm_init_hw() */
} bldc_driver_3pwm_t;

/**
 * Initialise driver structure with default values.
 *
 * @param driver  Pointer to driver structure.
 */
void bldc_driver_3pwm_init_struct(bldc_driver_3pwm_t *driver);

/**
 * Configure hardware (PWM channels, GPIOs).
 * Drives nSLEEP HIGH and INL HIGH.  Sets up nFAULT interrupt for logging.
 *
 * @param driver  Pointer to driver structure.
 * @return DRIVER_INIT_OK on success, DRIVER_INIT_FAILED on failure.
 */
int bldc_driver_3pwm_init_hw(bldc_driver_3pwm_t *driver);

/**
 * Enable the driver: set all duty cycles to 0 (motor at rest, bridge active).
 *
 * @param driver  Pointer to driver structure.
 */
void bldc_driver_3pwm_enable(bldc_driver_3pwm_t *driver);

/**
 * Disable the driver: zero all duty cycles (motor coasts, INL stays HIGH).
 *
 * @param driver  Pointer to driver structure.
 */
void bldc_driver_3pwm_disable(bldc_driver_3pwm_t *driver);

/**
 * Put the DRV8311H into low-power sleep: zero all duty cycles and drive
 * nSLEEP LOW. The bridge is disabled and the chip enters its sleep state
 * (~uA). Used by the controller power-off sequence. Call init again (or a
 * full reset/wake) to bring it back.
 *
 * @param driver  Pointer to driver structure.
 */
void bldc_driver_3pwm_sleep(bldc_driver_3pwm_t *driver);

/**
 * Write phase voltages to hardware.
 * Voltages are clamped to [0, voltage_limit] then converted to duty cycles.
 *
 * @param driver  Pointer to driver structure.
 * @param ua      Phase A voltage [V].
 * @param ub      Phase B voltage [V].
 * @param uc      Phase C voltage [V].
 */
void bldc_driver_3pwm_set_pwm(bldc_driver_3pwm_t *driver,
                               float ua, float ub, float uc);

/**
 * Read the raw logic levels of the DRV8311H status pins.
 * Any output pointer may be NULL. A value of -1 means the pin is not ready.
 *
 * @param nfault  nFAULT raw level (1=HIGH=OK, 0=LOW=fault asserted).
 * @param nsleep  nSLEEP raw level (1=HIGH=awake).
 * @param inl     INL raw level (1=HIGH=low-side bridge enabled).
 */
void bldc_driver_3pwm_get_status(int *nfault, int *nsleep, int *inl);

/**
 * Force-reassert the DRV8311H enable pins (INL HIGH, nSLEEP HIGH) and read
 * back the raw levels, returning the GPIO API return codes. Diagnostic helper
 * to distinguish "GPIO write does not take effect" (e.g. SPU/secure-domain or
 * pin-mux conflict) from "something else resets the pins after init".
 *
 * Any output pointer may be NULL.
 * @return 0 if all GPIO calls returned 0, -1 otherwise.
 */
int bldc_driver_3pwm_reassert_enable(int *nsleep_after, int *inl_after,
                                     int *sleep_cfg_rc, int *inl_cfg_rc,
                                     int *sleep_set_rc);

#endif /* BLDC_DRIVER_3PWM_H */
