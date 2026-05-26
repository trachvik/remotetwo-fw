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
 * Check whether the DRV8311H nFAULT pin is asserted.
 * Reads the GPIO level directly on every call — catches faults that were
 * already present at boot and would have been missed by the edge ISR.
 * Also returns true if a fault was already latched by the ISR.
 *
 * @param driver  Pointer to driver structure.
 * @return true if fault is active (nFAULT = LOW), false otherwise.
 */
bool bldc_driver_3pwm_is_fault(bldc_driver_3pwm_t *driver);

#endif /* BLDC_DRIVER_3PWM_H */
