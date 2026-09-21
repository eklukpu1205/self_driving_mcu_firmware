#include "actuator_hal.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Pin Definitions 
#define PIN_SERVO_PWM 14
#define PIN_MOTOR_PWM 15

// PWM Timing Calculations (50 Hz)
#define PWM_CLK_DIV 133.0f
#define PWM_WRAP_50HZ 19999

uint servo_slice;
uint motor_slice;

void actuator_hal_init(void) {
   
    // Assign pins to the PWM peripheral function
    gpio_set_function(PIN_SERVO_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_PWM, GPIO_FUNC_PWM);

    // Identify the hardware associated with each GPIO
    servo_slice = pwm_gpio_to_slice_num(PIN_SERVO_PWM);
    motor_slice = pwm_gpio_to_slice_num(PIN_MOTOR_PWM);

    // Configure the PWM
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLK_DIV);
    pwm_config_set_wrap(&config, PWM_WRAP_50HZ);

    // Load configuration into slices
    pwm_init(servo_slice, &config, false);
    pwm_init(motor_slice, &config, false);

    // Default actuators to neutral values
    pwm_set_gpio_level(PIN_SERVO_PWM, PULSE_NEUTRAL_US);
    pwm_set_gpio_level(PIN_MOTOR_PWM, PULSE_NEUTRAL_US);

    // Enable the PWM output slices
    pwm_set_enabled(servo_slice, true);
    pwm_set_enabled(motor_slice, true);
}

void set_raw_pulse(uint pin, uint16_t pulse_width_us) {
    
    // Bound pulse control values
    if (pulse_width_us < PULSE_ABSOLUTE_MIN_US) {
        pulse_width_us = PULSE_ABSOLUTE_MIN_US;
    }
    
    if (pulse_width_us > PULSE_ABSOLUTE_MAX_US) {
        pulse_width_us = PULSE_ABSOLUTE_MAX_US;
    }
    
    pwm_set_gpio_level(pin, pulse_width_us);
}

void actuator_hal_set_steering_rad(float angle_rad) {
    
    // Clamp input metric to mechanical limits
    if (angle_rad < STEERING_MIN_RAD){
        angle_rad = STEERING_MIN_RAD;
    }

    if (angle_rad > STEERING_MAX_RAD) {
        angle_rad = STEERING_MAX_RAD;
    }

    // Map radians to pulse width 
    float normalized = angle_rad / STEERING_MAX_RAD; 
    uint16_t pulse_us = (uint16_t) (PULSE_NEUTRAL_US + (normalized * (PULSE_SAFE_RIGHT_US - PULSE_NEUTRAL_US)));
    
    // Clamp control signal
    if (pulse_us < PULSE_SAFE_LEFT_US){
        pulse_us = PULSE_SAFE_LEFT_US;
    }
    if (pulse_us > PULSE_SAFE_RIGHT_US) {
        pulse_us = PULSE_SAFE_RIGHT_US;
    }

    set_raw_pulse(PIN_SERVO_PWM, pulse_us);
}

void actuator_hal_set_throttle_effort(float throttle_percent) {
    
    // Clamps input to [-1.0, 1.0]
    if (throttle_percent < -1.0f) {
        throttle_percent = -1.0f;
    }
    if (throttle_percent > 1.0f) {
        throttle_percent = 1.0f;
    }

    // Maps effort to standard RC PWM bounds [1000, 2000] us for the motor ESC
    uint16_t pulse_us = (uint16_t) (PULSE_NEUTRAL_US + (throttle_percent * 500.0f));
    
    // Clamps to motor ESC limits
    if (pulse_us < MOTOR_PULSE_MIN_US) {
        pulse_us = MOTOR_PULSE_MIN_US;
    }
    if (pulse_us > MOTOR_PULSE_MAX_US) {
        pulse_us = MOTOR_PULSE_MAX_US;
    }

    set_raw_pulse(PIN_MOTOR_PWM, pulse_us);
}

void actuator_hal_emergency_halt(void) {
    
      // Sets the motor and steering to neutral state
    set_raw_pulse(PIN_SERVO_PWM, PULSE_NEUTRAL_US);
    set_raw_pulse(PIN_MOTOR_PWM, PULSE_NEUTRAL_US);
}