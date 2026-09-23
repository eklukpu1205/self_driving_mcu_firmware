#include "actuator_hal.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// Pin Definitions 
#define PIN_SERVO_PWM 14


// PWM Timing Calculations (50 Hz)
#define PWM_CLK_DIV 133.0f
#define PWM_WRAP_50HZ 19999

uint servo_slice;

void actuator_hal_init(void) {
   
    // Assign pins to the PWM peripheral function
    gpio_set_function(PIN_SERVO_PWM, GPIO_FUNC_PWM);

    // Identify the hardware associated with each GPIO
    servo_slice = pwm_gpio_to_slice_num(PIN_SERVO_PWM);

    // Configure the PWM
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLK_DIV);
    pwm_config_set_wrap(&config, PWM_WRAP_50HZ);

    // Load configuration into slices
    pwm_init(servo_slice, &config, false);

    // Default actuators to neutral values
    pwm_set_gpio_level(PIN_SERVO_PWM, PULSE_NEUTRAL_US);

    // Enable the PWM output slices
    pwm_set_enabled(servo_slice, true);

    // Setup I2C communication with the motor driver
    i2c_comm_setup();

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
    
    // Utilize I2C to adjust the motor speed

}

void actuator_hal_emergency_halt(void) {
    
    // Sets the steering to neutral state
    set_raw_pulse(PIN_SERVO_PWM, PULSE_NEUTRAL_US);

    // Add control signal to set the motor speed to neutral
   

}

void i2c_comm_setup(void) {

    //Initializes the I2C communication

}