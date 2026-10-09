#include "actuator_hal.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <math.h>

#include "hardware/clocks.h"

// Pin Definitions 
#define PIN_SERVO_PWM 14

#define MOTOR_I2C_SDA_PIN 4
#define MOTOR_I2C_SCL_PIN 5
#define HI_WONDER_I2C_ADDR 0x34 // The motor driver module address
#define MOTOR_PWM_ADDR 0x1F



// PWM Timing Calculations (50 Hz)
#define PWM_WRAP_50HZ 19999

uint servo_slice;

void actuator_hal_init(void) {
   
    // Assign pins to the PWM peripheral function
    gpio_set_function(PIN_SERVO_PWM, GPIO_FUNC_PWM);

    // Identify the hardware associated with each GPIO
    servo_slice = pwm_gpio_to_slice_num(PIN_SERVO_PWM);

    // Configure the PWM
    pwm_config config = pwm_get_default_config();
    
    uint32_t sys_clk = clock_get_hz(clk_sys);
    float clk_div = (float) sys_clk / 1000000.0f;
    
    pwm_config_set_clkdiv(&config, clk_div);
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

void set_raw_pulse(int pin, uint16_t pulse_width_us) {
    
    // Bound pulse control values
    if (pulse_width_us < PULSE_ABSOLUTE_MIN_US) {
        pulse_width_us = PULSE_ABSOLUTE_MIN_US;
    }
    
    if (pulse_width_us > PULSE_ABSOLUTE_MAX_US) {
        pulse_width_us = PULSE_ABSOLUTE_MAX_US;
    }
    
    pwm_set_gpio_level(pin, pulse_width_us);
}

void actuator_hal_set_steering_pulse_us(uint16_t pulse_us) {
    // Bound pulse control values to safe limits
    if (pulse_us < PULSE_ABSOLUTE_MIN_US) {
        pulse_us = PULSE_ABSOLUTE_MIN_US;
    }
    if (pulse_us > PULSE_ABSOLUTE_MAX_US) {
        pulse_us = PULSE_ABSOLUTE_MAX_US;
    }
    
    set_raw_pulse(PIN_SERVO_PWM, pulse_us);
}

bool actuator_hal_set_throttle_effort(float throttle_percent) {

    if (!isfinite(throttle_percent)) {
        return;
    }
    
    // Utilize I2C to adjust the motor speed
    if (throttle_percent < -100.0f){
        throttle_percent = -100.0f;
    }
    if (throttle_percent > 100.0f) {
        throttle_percent = 100.0f;
    }


    int8_t pwm_val = (int8_t) throttle_percent;

    uint8_t motor_data[5] = {MOTOR_PWM_ADDR, (uint8_t)(-pwm_val), 0, (uint8_t)(pwm_val), 0};
 

    int result = i2c_write_blocking(i2c0, HI_WONDER_I2C_ADDR, motor_data, sizeof(motor_data), false);

    return result == (int)sizeof(motor_data);

}

void actuator_hal_emergency_halt(void) {
    
    // Sets the steering to neutral state
    set_raw_pulse(PIN_SERVO_PWM, PULSE_NEUTRAL_US);

    // Add control signal to set the motor speed to neutral
    actuator_hal_set_throttle_effort(0);
   

}

void i2c_comm_setup(void) {

    //Initializes the I2C communication
    i2c_init(i2c0, 100000);

    gpio_set_function(MOTOR_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MOTOR_I2C_SDA_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(MOTOR_I2C_SCL_PIN);
    gpio_pull_up(MOTOR_I2C_SDA_PIN);

}