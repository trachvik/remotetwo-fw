#include "bldc_motor.h"
#include "bldc_driver_3pwm.h"
#include "current_sense.h"
#include <string.h>
#include <math.h>
#include <arm_math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bldc_motor, LOG_LEVEL_DBG);

/* Helper macros */
#define _ISSET(x) ((x) != NOT_SET)
#define _CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Default configuration values */
#define DEF_POWER_SUPPLY 12.0f
#define DEF_PID_VEL_P 0.5f
#define DEF_PID_VEL_I 10.0f
#define DEF_PID_VEL_D 0.0f
#define DEF_PID_VEL_RAMP 1000.0f
#define DEF_PID_VEL_LIMIT 12.0f
#define DEF_P_ANGLE_P 20.0f
#define DEF_VEL_LIM 20.0f
#define DEF_VEL_FILTER_TF 0.005f
#define DEF_PID_CURR_P 0.5f       /* Reduced: was 3.0f (too aggressive) */
#define DEF_PID_CURR_I 5.0f       /* Drastically reduced: was 300.0f (caused oscillation) */
#define DEF_PID_CURR_D 0.0f
#define DEF_PID_CURR_RAMP 10.0f   /* Reduced: was 1000.0f (limited voltage slew rate to ~3V per cycle max) */
#define DEF_CURR_FILTER_TF 0.005f
#define DEF_MOTION_DOWNSAMPLE 0
#define DEF_MON_DOWNSAMPLE 100

/**
 * Time functions
 */
unsigned long micros(void)
{
    return k_cyc_to_us_floor64(k_cycle_get_64());
}

/*void delay_ms(unsigned long ms)
{
    k_msleep(ms);
}*/

/**
 * Sensor interface functions (implemented in as5048a.c)
 */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);
extern float sensor_get_velocity(sensor_t *sensor); // TO DO - not implemented yet, is this needed?
extern bool sensor_needs_search(sensor_t *sensor);

/**
 * Driver interface wrapper functions
 */
void driver_set_pwm(bldc_driver_t *driver, float ua, float ub, float uc)
{
    if (driver == NULL) return;

    bldc_driver_3pwm_t *drv = (bldc_driver_3pwm_t *)driver;
    bldc_driver_3pwm_set_pwm(drv, ua, ub, uc);
}

void driver_enable(bldc_driver_t *driver)
{
    if (driver == NULL) return;

    bldc_driver_3pwm_t *drv = (bldc_driver_3pwm_t *)driver;
    bldc_driver_3pwm_enable(drv);
}

void driver_disable(bldc_driver_t *driver)
{
    if (driver == NULL) return;

    bldc_driver_3pwm_t *drv = (bldc_driver_3pwm_t *)driver;
    bldc_driver_3pwm_disable(drv);
}

bool driver_is_initialized(bldc_driver_t *driver)
{
    if (driver == NULL) return false;

    bldc_driver_3pwm_t *drv = (bldc_driver_3pwm_t *)driver;
    return drv->initialized;
}

float driver_get_voltage_limit(bldc_driver_t *driver)
{
    if (driver == NULL) return 0.0f;

    bldc_driver_3pwm_t *drv = (bldc_driver_3pwm_t *)driver;
    return drv->voltage_limit;
}

float driver_get_voltage_power_supply(bldc_driver_t *driver)
{
    if (driver == NULL) return 0.0f;

    bldc_driver_3pwm_t *drv = (bldc_driver_3pwm_t *)driver;
    return drv->voltage_power_supply;
}

/**
 * Current sense stubs
 */
// void current_sense_enable(current_sense_t *cs)
// {
//     /* Stub - no current sensing yet */
// }

// void current_sense_disable(current_sense_t *cs)
// {
//     /* Stub - no current sensing yet */
// }

// bool current_sense_is_initialized(current_sense_t *cs)
// {
//     /* Stub - no current sensing yet */
//     return false;
// }

// int current_sense_driver_align(current_sense_t *cs, float voltage, int8_t modulation_centered)
// {
//     /* Stub - no current sensing yet */
//     return 1;
// }

/* Trapezoid maps */
/*static const int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, _HIGH_IMPEDANCE, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, _HIGH_IMPEDANCE, -1}
};

static const int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, 1, 1},
    {-1, _HIGH_IMPEDANCE, 1},
    {-1, -1, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, -1, -1},
    {1, _HIGH_IMPEDANCE, -1},
    {1, 1, -1}
};*/

/**
 * PID controller initialization
 */
void pid_controller_init(pid_controller_t *pid, float p, float i, float d, float ramp, float limit)
{
    if (pid == NULL) return;
    
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->Ts = NOT_SET;  /* adaptive by default; set >0 for fixed-rate deterministic mode */
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = 0;
}

/**
 * PID controller operator
 */
float pid_controller_operator(pid_controller_t *pid, float error)
{
    if (pid == NULL) return 0.0f;

    /* Sampling time: use fixed Ts when set (deterministic FOC loop),
     * otherwise measure elapsed time with micros() (SimpleFOC adaptive mode). */
    float ts;
    if (pid->Ts > 0.0f) {
        ts = pid->Ts;
    } else {
        unsigned long timestamp = micros();
        ts = (timestamp - pid->timestamp_prev) * 1e-6f;
        if (ts <= 0.0f || ts > 0.5f) ts = 1e-3f;
        pid->timestamp_prev = timestamp;
    }

    /* Proportional term */
    float proportional = pid->p * error;

    /* Integral term (trapezoidal), anti-windup clamp to output limit */
    float integral = pid->integral_prev + pid->i * ts * 0.5f * (error + pid->error_prev);
    integral = _CONSTRAIN(integral, -pid->limit, pid->limit);

    /* Derivative term */
    float derivative = pid->d * (error - pid->error_prev) / ts;

    /* Calculate output */
    float output = proportional + integral + derivative;
    output = _CONSTRAIN(output, -pid->limit, pid->limit);

    /* Apply ramp limit */
    if (pid->output_ramp > 0) {
        float output_rate = (output - pid->output_prev) / ts;
        if (output_rate > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * ts;
        } else if (output_rate < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * ts;
        }
    }

    /* Save state */
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    /* timestamp_prev already updated at the start in adaptive mode */

    return output;
}

/**
 * PID controller reset
 */
void pid_controller_reset(pid_controller_t *pid)
{
    if (pid == NULL) return;
    
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = 0;
}

/**
 * Low pass filter initialization
 */
void lowpass_filter_init(lowpass_filter_t *lpf, float tf)
{
    if (lpf == NULL) return;
    
    lpf->tf = tf;
    lpf->Ts = NOT_SET;  /* adaptive by default */
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = 0;
}

/**
 * Low pass filter operator
 */
float lowpass_filter_operator(lowpass_filter_t *lpf, float x)
{
    if (lpf == NULL) return x;

    /* Sampling time: fixed Ts or adaptive via micros(). */
    float dt;
    if (lpf->Ts > 0.0f) {
        dt = lpf->Ts;
    } else {
        unsigned long timestamp = micros();
        dt = (timestamp - lpf->timestamp_prev) * 1e-6f;
        if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;
        lpf->timestamp_prev = timestamp;
    }

    /* alpha = Tf / (Tf + dt)  — identical to SimpleFOC LowPassFilter */
    float alpha = lpf->tf / (lpf->tf + dt);
    float y = alpha * lpf->y_prev + (1.0f - alpha) * x;

    lpf->y_prev = y;
    return y;
}

/**
 * Normalize angle to [0, 2PI]
 */
static float normalize_angle(float angle)
{
    /* fmodf not in CMSIS DSP, keep standard implementation */
    float a = fmodf(angle, _2PI);
    return a >= 0.0f ? a : (a + _2PI);
}

/**
 * Initialize BLDC motor structure
 */
void bldc_motor_init_struct(bldc_motor_t *motor, int pp, float r, float kv, float l)
{
    if (motor == NULL) return;
    
    memset(motor, 0, sizeof(bldc_motor_t));
    
    /* Motor physical parameters */
    motor->pole_pairs = pp;
    motor->phase_resistance = r;
    motor->kv_rating = _ISSET(kv) ? kv : NOT_SET;
    motor->phase_inductance = l;
    
    /* Default configuration */
    motor->voltage_sensor_align = 6.0f;  /* Higher voltage for better alignment */
    motor->velocity_index_search = 1.0f;
    motor->voltage_limit = DEF_POWER_SUPPLY;
    motor->current_limit = NOT_SET;
    motor->velocity_limit = DEF_VEL_LIM;
    
    motor->zero_electric_angle = NOT_SET;
    motor->sensor_direction = DIR_UNKNOWN;
    motor->sensor_offset = 0.0f;
    
    /* Control configuration */
    motor->torque_controller = VOLTAGE;
    motor->controller = TORQUE;
    motor->foc_modulation = SPACE_VECTOR_PWM;  /* SVPWM: min/max zero-seq, lower THD */
    motor->modulation_centered = 1;
    
    /* Initialize controllers */
    pid_controller_init(&motor->pid_current_q, DEF_PID_CURR_P, DEF_PID_CURR_I, 
                       DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY);
    pid_controller_init(&motor->pid_current_d, DEF_PID_CURR_P, DEF_PID_CURR_I,
                       DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY);
    pid_controller_init(&motor->pid_velocity, DEF_PID_VEL_P, DEF_PID_VEL_I,
                       DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT);
    pid_controller_init(&motor->p_angle, DEF_P_ANGLE_P, 0.0f, 0.0f, 0.0f, DEF_VEL_LIM);
    
    lowpass_filter_init(&motor->lpf_current_q, DEF_CURR_FILTER_TF);
    lowpass_filter_init(&motor->lpf_current_d, DEF_CURR_FILTER_TF);
    lowpass_filter_init(&motor->lpf_velocity, DEF_VEL_FILTER_TF);
    lowpass_filter_init(&motor->lpf_angle, 0.0f);
    
    motor->motion_downsample = DEF_MOTION_DOWNSAMPLE;
    motor->monitor_downsample = DEF_MON_DOWNSAMPLE;
    
    motor->motor_status = MOTOR_UNINITIALIZED;
    motor->enabled = 0;
}

/**
 * Link driver to motor
 */
void bldc_motor_link_driver(bldc_motor_t *motor, bldc_driver_t *driver)
{
    if (motor == NULL) return;
    motor->driver = driver;
}

/**
 * Link sensor to motor
 */
void bldc_motor_link_sensor(bldc_motor_t *motor, sensor_t *sensor)
{
    if (motor == NULL) return;
    motor->sensor = sensor;
}

/**
 * Link current sense to motor
 */
void bldc_motor_link_current_sense(bldc_motor_t *motor, current_sense_t *current_sense)
{
    if (motor == NULL) return;
    motor->current_sense = current_sense;
}

/**
 * Calculate shaft angle
 */
float bldc_motor_shaft_angle(bldc_motor_t *motor)
{
    if (motor == NULL || motor->sensor == NULL) return 0.0f;
    
    float angle = sensor_get_angle(motor->sensor) - motor->sensor_offset;
    return (float)motor->sensor_direction * angle;
}

/**
 * Calculate shaft velocity
 */
float bldc_motor_shaft_velocity(bldc_motor_t *motor)
{
    if (motor == NULL || motor->sensor == NULL) return 0.0f;
    
    float vel = sensor_get_velocity(motor->sensor);
    return lowpass_filter_operator(&motor->lpf_velocity, 
                                   (float)motor->sensor_direction * vel);
}

/**
 * Calculate electrical angle.
 * The mechanical angle is already smoothed by the sensor-layer tracking observer
 * (see tmag5170_sensor.c), so commutation gets a clean angle here without an
 * extra low-pass that would add torque-angle lag. */
float bldc_motor_electrical_angle(bldc_motor_t *motor)
{
    if (motor == NULL) return 0.0f;

    float angle = bldc_motor_shaft_angle(motor) * motor->pole_pairs - motor->zero_electric_angle;
    return normalize_angle(angle);
}

/**
 * Initialize motor hardware
 */
int bldc_motor_init(bldc_motor_t *motor)
{
    if (motor == NULL) return 0;
    
    if (motor->driver == NULL || !driver_is_initialized(motor->driver)) {
        motor->motor_status = MOTOR_INIT_FAILED;
        return 0;
    }
    
    motor->motor_status = MOTOR_INITIALIZING;
    
    /* Sanity check for voltage limit */
    float driver_voltage_limit = driver_get_voltage_limit(motor->driver);
    if (motor->voltage_limit > driver_voltage_limit) {
        motor->voltage_limit = driver_voltage_limit;
    }
    if (motor->voltage_sensor_align > motor->voltage_limit) {
        motor->voltage_sensor_align = motor->voltage_limit;
    }
    
    /* Update controller limits */
    if (motor->current_sense != NULL) {
        motor->pid_current_q.limit = motor->voltage_limit;
        motor->pid_current_d.limit = motor->voltage_limit;
    }
    
    if (_ISSET(motor->phase_resistance) /*|| motor->torque_controller != VOLTAGE*/) {
        motor->pid_velocity.limit = motor->current_limit;
    } else {
        motor->pid_velocity.limit = motor->voltage_limit;
    }
    
    motor->p_angle.limit = motor->velocity_limit;
    
    /* Set default direction for open loop if no sensor */
    if (motor->sensor == NULL) {
        if ((motor->controller == VELOCITY_OPENLOOP || motor->controller == ANGLE_OPENLOOP) &&
            motor->sensor_direction == DIR_UNKNOWN) {
            motor->sensor_direction = DIR_CW;
        }
    }
    
    k_msleep(500);  /* Short delay for stability */
    bldc_motor_enable(motor);
    k_msleep(500);
    
    motor->motor_status = MOTOR_UNCALIBRATED;
    return 1;
}

/**
 * Disable motor
 */
void bldc_motor_disable(bldc_motor_t *motor)
{
    if (motor == NULL || motor->driver == NULL) return;
    
    //if (motor->current_sense != NULL) {
        //(motor->current_sense);
    //}
    
    driver_set_pwm(motor->driver, 0.0f, 0.0f, 0.0f);
    driver_disable(motor->driver);
    
    motor->enabled = 0;
}

/**
 * Enable motor
 */
void bldc_motor_enable(bldc_motor_t *motor)
{
    if (motor == NULL || motor->driver == NULL) return;
    
    driver_enable(motor->driver);
    driver_set_pwm(motor->driver, 0.0f, 0.0f, 0.0f);
    
    /*if (motor->current_sense != NULL) {
        current_sense_enable(motor->current_sense);
    }*/
    
    /* Reset controllers */
    pid_controller_reset(&motor->pid_velocity);
    pid_controller_reset(&motor->p_angle);
    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);
    
    motor->enabled = 1;
}

/**
 * Set phase voltage using FOC (simplified version)
 */
void bldc_motor_set_phase_voltage(bldc_motor_t *motor, float uq, float ud, float angle_el)
{
    if (motor == NULL || motor->driver == NULL) return;
    
    /* Calculate sin and cos for Park transform */
    float ca = cosf(angle_el);
    float sa = sinf(angle_el);
    
    /* Inverse Park transform using ARM CMSIS DSP */
    float u_alpha, u_beta;
    arm_inv_park_f32(ud, uq, &u_alpha, &u_beta, sa, ca);
    
    motor->u_alpha = u_alpha;
    motor->u_beta = u_beta;
    
    /* Inverse Clarke transform using ARM CMSIS DSP */
    float ua, ub;
    arm_inv_clarke_f32(u_alpha, u_beta, &ua, &ub);
    motor->ua = ua;
    motor->ub = ub;
    motor->uc = -(ua + ub);  /* Ia + Ib + Ic = 0 */
    
    /* Modulation strategy: SVPWM or SINE_PWM with half-VDC centering.
     *
     * SVPWM zero-sequence: v_offset = Vdc/2 - (max+min)/2
     * This eliminates even harmonics and reduces current THD by ~30% compared
     * to constant-midpoint SPWM.  Same fundamental torque, lower ripple.
     * Extends linear modulation range by 15.5% (2/sqrt(3) vs 1) at no cost.
     *
     * The centre MUST be half the DC BUS (what maps to 50% duty in
     * driver_set_pwm), not motor->voltage_limit. motor->voltage_limit is the
     * cap on the |U| vector magnitude and is deliberately only 0.58*Vdc for
     * SVPWM; centering on it puts the waveform at 29% duty, so every negative
     * phase excursion beyond Vdc*0.58/2 gets clamped to 0 V by the driver while
     * the top 21% of the bus is never used. The asymmetric clip is a hard
     * nonlinearity the current PI winds up against. */
    float v_bus = driver_get_voltage_power_supply(motor->driver);
    if (v_bus <= 0.0f) v_bus = motor->voltage_limit;

    if (motor->foc_modulation == SPACE_VECTOR_PWM) {
        float v_max = motor->ua;
        if (motor->ub > v_max) v_max = motor->ub;
        if (motor->uc > v_max) v_max = motor->uc;
        float v_min = motor->ua;
        if (motor->ub < v_min) v_min = motor->ub;
        if (motor->uc < v_min) v_min = motor->uc;
        float v_offset = 0.5f * v_bus - 0.5f * (v_max + v_min);
        motor->ua += v_offset;
        motor->ub += v_offset;
        motor->uc += v_offset;
    } else if (motor->modulation_centered) {
        /* SINE_PWM: constant half-VDC shift */
        float mid = 0.5f * v_bus;
        motor->ua += mid;
        motor->ub += mid;
        motor->uc += mid;
    }

    /* Set PWM */
    driver_set_pwm(motor->driver, motor->ua, motor->ub, motor->uc);
}

/**
 * Initialize FOC algorithm with sensor alignment
 */
int bldc_motor_init_foc(bldc_motor_t *motor)
{
    if (motor == NULL) return 0;

    int exit_flag = 1;

    motor->motor_status = MOTOR_CALIBRATING;

    /* No sensor -> no sensored FOC possible on this build. */
    if (motor->sensor == NULL) {
        LOG_ERR("MOT: no sensor - sensored FOC not possible");
        motor->motor_status = MOTOR_CALIB_FAILED;
        bldc_motor_disable(motor);
        return 0;
    }

    /* ==================== alignSensor() (SimpleFOC) ==================== */
    LOG_INF("MOT: Align sensor.");

    /* Refresh the sensor and make sure it does not require an index search. */
    sensor_update(motor->sensor);
    if (sensor_needs_search(motor->sensor)) {
        LOG_ERR("MOT: sensor index search required (unsupported)");
        motor->motor_status = MOTOR_CALIB_FAILED;
        bldc_motor_disable(motor);
        return 0;
    }

    /* Fixed voltage used for the alignment (SimpleFOC v3.1 behaviour). */
    float voltage_align = motor->voltage_sensor_align;

    /* ---- Natural direction detection (only if unknown) ---- */
    if (motor->sensor_direction == DIR_UNKNOWN) {
        /* Move one electrical revolution forward. */
        for (int i = 0; i <= 500; i++) {
            float angle = _3PI_2 + _2PI * (float)i / 500.0f;
            bldc_motor_set_phase_voltage(motor, voltage_align, 0.0f, angle);
            sensor_update(motor->sensor);
            k_msleep(2);
        }
        /* Take an angle in the middle. */
        sensor_update(motor->sensor);
        float mid_angle = sensor_get_angle(motor->sensor);

        /* Move one electrical revolution backwards. */
        for (int i = 500; i >= 0; i--) {
            float angle = _3PI_2 + _2PI * (float)i / 500.0f;
            bldc_motor_set_phase_voltage(motor, voltage_align, 0.0f, angle);
            sensor_update(motor->sensor);
            k_msleep(2);
        }
        sensor_update(motor->sensor);
        float end_angle = sensor_get_angle(motor->sensor);
        bldc_motor_set_phase_voltage(motor, 0.0f, 0.0f, 0.0f);
        k_msleep(200);

        /* Determine the direction the sensor moved. Wrap the delta into [-PI,PI]:
         * the absolute sensor angle can straddle the 0/2*PI boundary, so a naive
         * subtraction reports a near-full-turn move that fails the PP check and
         * needlessly disables the motor even though the rotor moved correctly. */
        float delta = mid_angle - end_angle;
        while (delta >  _PI) delta -= _2PI;
        while (delta < -_PI) delta += _2PI;
        float moved = fabsf(delta);
        if (moved < MIN_ANGLE_DETECT_MOVEMENT) {
            LOG_ERR("MOT: Failed to notice movement");
            motor->motor_status = MOTOR_CALIB_FAILED;
            bldc_motor_disable(motor);
            return 0;   /* failed calibration */
        } else if (delta < 0.0f) {
            LOG_INF("MOT: sensor_direction == CCW");
            motor->sensor_direction = DIR_CCW;
        } else {
            LOG_INF("MOT: sensor_direction == CW");
            motor->sensor_direction = DIR_CW;
        }

        /* Pole pair number sanity check. */
        float moved_electr = moved * (float)motor->pole_pairs;
        if (fabsf(moved_electr - _2PI) > 0.5f) {
            LOG_WRN("MOT: PP check: fail - estimated pp: %.1f",
                    (double)(_2PI / moved));
            
            motor->motor_status = MOTOR_CALIB_FAILED;
            bldc_motor_disable(motor);
            return 0; 
            
        } else {
            LOG_INF("MOT: PP check: OK!");
        }
    } else {
        LOG_INF("MOT: Skip dir calib.");
    }

    /* ---- Zero electric angle offset (only if unknown) ---- */
    if (!_ISSET(motor->zero_electric_angle)) {
        LOG_INF("MOT: Aligning zero electric angle (rotating-field average)...");

        /* Single-point alignment locks ONE pole and is corrupted by cogging: the
         * detent pulls the rotor off the commanded angle, and which of the pole
         * pairs it locks onto depends on the rest position at boot, so the offset
         * differs run-to-run (rough/self-spin on some boots). Instead rotate the
         * field through a full MECHANICAL revolution (pole_pairs electrical revs)
         * forward and backward and take the circular mean of the per-step offset:
         * cogging averages out over a revolution and friction hysteresis cancels
         * between the two directions. Reduces to the old formula for one sample.
         *
         * The mean must be circular (atan2 of the summed sin/cos), not a plain
         * arithmetic mean of the angles - the per-step offset straddles the
         * 0/2*PI wrap, and averaging across that wrap lands the result half a
         * turn away from the truth. */
        motor->zero_electric_angle = 0.0f; /* electrical_angle() now returns raw shaft*pp */

        const int   steps_per_rev = 100;
        const int   total_steps   = motor->pole_pairs * steps_per_rev;
        const float start_field   = _3PI_2;
        float sin_sum = 0.0f, cos_sum = 0.0f;

        /* Pre-lock at the start field and let the rotor settle. */
        bldc_motor_set_phase_voltage(motor, voltage_align, 0.0f, start_field);
        k_msleep(300);

        for (int i = 0; i <= total_steps; i++) {          /* forward sweep */
            float field = start_field + _2PI * (float)i / (float)steps_per_rev;
            bldc_motor_set_phase_voltage(motor, voltage_align, 0.0f, field);
            k_msleep(3);
            sensor_update(motor->sensor);
            float off = normalize_angle(bldc_motor_electrical_angle(motor) - field);
            sin_sum += sinf(off);
            cos_sum += cosf(off);
        }
        for (int i = total_steps; i >= 0; i--) {          /* backward sweep */
            float field = start_field + _2PI * (float)i / (float)steps_per_rev;
            bldc_motor_set_phase_voltage(motor, voltage_align, 0.0f, field);
            k_msleep(3);
            sensor_update(motor->sensor);
            float off = normalize_angle(bldc_motor_electrical_angle(motor) - field);
            sin_sum += sinf(off);
            cos_sum += cosf(off);
        }

        /* Stop everything. */
        bldc_motor_set_phase_voltage(motor, 0.0f, 0.0f, 0.0f);
        k_msleep(200);

        motor->zero_electric_angle =
            normalize_angle(start_field + atan2f(sin_sum, cos_sum));

        LOG_INF("MOT: Zero elec. angle (avg): %.3f", (double)motor->zero_electric_angle);
    } else {
        LOG_INF("MOT: Skip offset calib.");
    }

    motor->motor_status = MOTOR_READY;
    return exit_flag;
}

/**
 * FOC loop - SimpleFOC style
 * Update sensor and electrical angle, then apply the voltages calculated in move()
 */
void bldc_motor_loop_foc(bldc_motor_t *motor) // TO DO - anticogging conpensation
{
    if (motor == NULL) return;
    
    /* Sensor is read (and cached) externally before calling this function.
     * sensor_update() must be called once per cycle by the caller. */
    
    /* If disabled, do nothing */
    if (!motor->enabled) {
        return;
    }
    
    /* Calculate electrical angle from sensor */
    motor->electrical_angle = bldc_motor_electrical_angle(motor);

    /* Torque source: voltage (open-loop) or FOC current (closed current loop). */
    if (motor->torque_controller == FOC_CURRENT) {
        static unsigned int foc_cyc = 0;

        if (current_sense_is_ready()) {
            /* --- Real FOC current control (SimpleFOC loopFOC foc_current case) --- */
            float id_meas = 0.0f, iq_meas = 0.0f;
            current_sense_get_foc_currents(motor->electrical_angle, &id_meas, &iq_meas);

            /* IMPORTANT: do not print per-cycle DATA from this 5 kHz control loop.
             * Blocking UART prints here destroy loop timing and destabilize FOC.
             * Telemetry streaming is handled in haptic.c at a throttled rate. */

            motor->current.q = lowpass_filter_operator(&motor->lpf_current_q, iq_meas);
            motor->current.d = lowpass_filter_operator(&motor->lpf_current_d, id_meas);

            motor->voltage.q = pid_controller_operator(&motor->pid_current_q,
                                                        motor->current_sp - motor->current.q);
            motor->voltage.d = pid_controller_operator(&motor->pid_current_d,
                                                        -motor->current.d);
            foc_cyc++;
        } else {
            /* Current sense NOT available. Do NOT pretend we are in current
             * control. Drive a voltage so the knob still works, but SHOUT about
             * it every second so a broken current loop can never hide. */
            motor->voltage.q = _CONSTRAIN(motor->target, -motor->voltage_limit, motor->voltage_limit);
            motor->voltage.d = 0.0f;
            if ((foc_cyc++ % 1000u) == 0u) {
                LOG_WRN("### CURRENT CONTROL UNAVAILABLE - SAADC not ready - running VOLTAGE fallback (NOT current control!) ###");
            }
        }
    } else if (current_sense_is_ready()) {
        /* VOLTAGE mode: still measure Iq/Id (LUT logging, telemetry). Do not
         * close the current PI — a current source on a cogging spring is an
         * undamped ~28 Hz oscillator. Back-EMF through R damps instead. */
        float id_meas = 0.0f, iq_meas = 0.0f;
        current_sense_get_foc_currents(motor->electrical_angle, &id_meas, &iq_meas);
        motor->current.q = lowpass_filter_operator(&motor->lpf_current_q, iq_meas);
        motor->current.d = lowpass_filter_operator(&motor->lpf_current_d, id_meas);
    }

    /* Apply the phase voltages using the fresh electrical angle. */
    bldc_motor_set_phase_voltage(motor, motor->voltage.q, motor->voltage.d, motor->electrical_angle);
}

/**
 * Align the current-sense polarity to the driver (SimpleFOC driverAlign).
 * Drives the alpha (phase-A) axis with a known positive voltage and checks that
 * the measured phase-A current is positive; flips the gain sign if not.
 */
int bldc_motor_align_current_sense(bldc_motor_t *motor)
{
    if (motor == NULL || motor->driver == NULL) return 0;
    if (!current_sense_is_ready()) {
        LOG_WRN("current-sense align skipped: SAADC not ready");
        return 0;
    }

    float v = motor->voltage_sensor_align;

    /* Ramp a voltage onto the alpha axis: set_phase_voltage(uq=0, ud=v, angle=0)
     * => u_alpha = v, so phase A is driven high vs B/C and phase-A current
     * should be physically positive. */
    for (int i = 0; i <= 100; i++) {
        bldc_motor_set_phase_voltage(motor, 0.0f, v * (float)i / 100.0f, 0.0f);
        k_msleep(2);
    }
    k_msleep(200);

    /* Average phase-A current over many reads to beat noise. */
    float ia = 0.0f;
    const int rounds = 200;
    for (int i = 0; i < rounds; i++) {
        phase_current_t c = current_sense_get_phase_currents();
        ia += c.a;
        k_msleep(1);
    }
    ia /= (float)rounds;

    /* Ramp the voltage back down and stop. */
    for (int i = 100; i >= 0; i--) {
        bldc_motor_set_phase_voltage(motor, 0.0f, v * (float)i / 100.0f, 0.0f);
        k_msleep(2);
    }
    bldc_motor_set_phase_voltage(motor, 0.0f, 0.0f, 0.0f);

    /* Note: ia here was read WITH the current default sign applied (+1). */
    float raw_ia = ia * current_sense_get_gain_sign();
    float sign = (raw_ia < 0.0f) ? -1.0f : 1.0f;
    current_sense_set_gain_sign(sign);

    LOG_INF("current-sense align: Ia(alpha-drive)=%.3f A -> gain_sign=%+.0f",
            (double)raw_ia, (double)sign);
    return 1;
}

/**
 * Configure the closed-loop current (Id/Iq) PI regulators and measurement LPFs
 * from motor physical parameters using the SimpleFOC bandwidth (IMC) method:
 *
 *   ωc  = 2π · bw_hz
 *   Kp  = phase_inductance · ωc    [V/A]
 *   Ki  = phase_resistance  · ωc   [V/(A·s)]
 *   LPF_Tf = 1/(2π · lpf_cutoff_hz)
 *   Ts  = 1/foc_hz  (fixed, deterministic)
 *
 * Typical starting values for this motor (R=5.6Ω, L=100µH):
 *   bw_hz=150 (3% of 5kHz) → Kp≈0.094 V/A, Ki≈5278 V/(A·s)
 *   lpf_cutoff=500Hz → Tf≈318µs  (lower to ~200Hz if motor buzzes)
 *
 * Must be called AFTER voltage_limit is set and BEFORE enabling FOC_CURRENT.
 */
void bldc_motor_configure_current_loop(bldc_motor_t *motor,
                                       float bw_hz,
                                       float foc_hz,
                                       float lpf_cutoff_hz)
{
    if (motor == NULL) return;

    float omega_c  = _2PI * bw_hz;
    float kp       = motor->phase_inductance * omega_c; /* P = L · ωc */
    float ki       = motor->phase_resistance  * omega_c; /* I = R · ωc */
    float Ts       = 1.0f / foc_hz;
    float lpf_tf   = 1.0f / (_2PI * lpf_cutoff_hz);

    /* Current PI: no derivative, no output ramp (0 = disabled).
     * Output limited to ±voltage_limit (already set by caller). */
    pid_controller_init(&motor->pid_current_q, kp, ki, 0.0f, 0.0f, motor->voltage_limit);
    motor->pid_current_q.Ts = Ts;
    pid_controller_init(&motor->pid_current_d, kp, ki, 0.0f, 0.0f, motor->voltage_limit);
    motor->pid_current_d.Ts = Ts;

    /* Measurement LPF on Iq and Id (applied before the PI).
     * Filters the ±4 mA ADC noise before it reaches the integrator. */
    lowpass_filter_init(&motor->lpf_current_q, lpf_tf);
    motor->lpf_current_q.Ts = Ts;
    lowpass_filter_init(&motor->lpf_current_d, lpf_tf);
    motor->lpf_current_d.Ts = Ts;

    LOG_INF("current loop configured: bw=%.0f Hz  foc=%.0f Hz  "
            "Kp=%.4f V/A  Ki=%.1f V/(A*s) | "
            "LPF fc=%.0f Hz  Tf=%.0f us  Ts=%.0f us",
            (double)bw_hz, (double)foc_hz,
            (double)kp,    (double)ki,
            (double)lpf_cutoff_hz,
            (double)(lpf_tf * 1e6f),
            (double)(Ts    * 1e6f));
}

void bldc_motor_reset_current_loop_state(bldc_motor_t *motor)
{
    if (motor == NULL) return;

    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);
    motor->lpf_current_q.y_prev = 0.0f;
    motor->lpf_current_d.y_prev = 0.0f;
    motor->current.q = 0.0f;
    motor->current.d = 0.0f;
    motor->voltage.q = 0.0f;
    motor->voltage.d = 0.0f;
    motor->current_sp = 0.0f;
}

void bldc_motor_set_bridge_hiz(bldc_motor_t *motor, bool hiz)
{
    if (motor == NULL || motor->driver == NULL) return;

    bldc_driver_3pwm_t *drv = (bldc_driver_3pwm_t *)motor->driver;

    /* Zero high-side PWM before the INL edge so a rising enable cannot
     * apply the last SVPWM vector into an open bridge. */
    driver_set_pwm(motor->driver, 0.0f, 0.0f, 0.0f);
    motor->voltage.q = 0.0f;
    motor->voltage.d = 0.0f;
    motor->current_sp = 0.0f;
    bldc_driver_3pwm_set_lowside_enable(drv, !hiz);
}

/**
 * Motion control loop - SimpleFOC style
 * Calculate voltages based on control mode and target
 */
void bldc_motor_move(bldc_motor_t *motor, float target)
{
    if (motor == NULL) return;
    
    /* Set target if provided */
    if (_ISSET(target)) {
        motor->target = target;
    }
    
    /* Update shaft angle and velocity for calculations */
    motor->shaft_angle = bldc_motor_shaft_angle(motor);
    motor->shaft_velocity = bldc_motor_shaft_velocity(motor);
    
    /* If disabled, do nothing */
    if (!motor->enabled) return;
    
    /* Calculate back-EMF voltage if KV rating available */
    #define _SQRT3 1.732050808f
    #define _RPM_TO_RADS 0.10471975512f
    if (_ISSET(motor->kv_rating)) {
        motor->voltage_bemf = motor->shaft_velocity / (motor->kv_rating * _SQRT3) / _RPM_TO_RADS;
    }
    
    /* Motion control based on controller type */
    switch (motor->controller) {
        case TORQUE:
            if (motor->torque_controller == VOLTAGE) {
                /* Pure voltage control - target is voltage directly */
                motor->voltage.q = _CONSTRAIN(motor->target, -motor->voltage_limit, motor->voltage_limit);
                motor->voltage.d = 0.0f;
            } else {
                /* Current control mode - target is current in Amps */
                motor->current_sp = motor->target;
            }
            break;
            
        case VELOCITY:
        {
            /* Velocity control with PID */
            float velocity_error = motor->target - motor->shaft_velocity;
            
            /* Output of velocity PID is the target for the torque controller (current or voltage) */
            float pid_output = pid_controller_operator(&motor->pid_velocity, velocity_error);
            
            if (motor->torque_controller == VOLTAGE) {
                motor->voltage.q = _CONSTRAIN(pid_output, -motor->voltage_limit, motor->voltage_limit);
                motor->voltage.d = 0.0f;
            } else {
                motor->current_sp = pid_output;
            }
            break;
        }
            
        case ANGLE:
        {
            /* Angle control with cascaded PID */
            float angle_error = motor->target - motor->shaft_angle;
            
            /* Output of angle P-controller is target velocity */
            float target_velocity = pid_controller_operator(&motor->p_angle, angle_error);
            
            /* Calculate velocity error */
            float velocity_error = target_velocity - motor->shaft_velocity;
            
            /* Output of velocity PID is target torque (current or voltage) */
            float pid_output = pid_controller_operator(&motor->pid_velocity, velocity_error);
            
            if (motor->torque_controller == VOLTAGE) {
                motor->voltage.q = _CONSTRAIN(pid_output, -motor->voltage_limit, motor->voltage_limit);
                motor->voltage.d = 0.0f;
            } else {
                motor->current_sp = pid_output;
            }
            break;
        }
            
        default:
            /* Open loop modes can be added later if needed */
            break;
    }
}
