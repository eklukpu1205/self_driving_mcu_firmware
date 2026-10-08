#include "actuator_hal.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

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
    float normalized = (angle_rad - STEERING_MIN_RAD) / (STEERING_MAX_RAD - STEERING_MIN_RAD); 
    uint16_t pulse_us = (uint16_t) (PULSE_SAFE_LEFT_US + (normalized * (PULSE_SAFE_RIGHT_US - PULSE_SAFE_LEFT_US)));
    
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
    if (throttle_percent < -100.0f){
        throttle_percent = -100.0f;
    }
    if (throttle_percent > 100.0f) {
        throttle_percent = 100.0f;
    }


    int8_t pwm_val = (int8_t) throttle_percent;

    uint8_t motor_data[2];
    motor_data[0] = MOTOR_PWM_ADDR;
    motor_data[1] = (uint8_t) pwm_val;

    i2c_write_blocking(i2c0, HI_WONDER_I2C_ADDR, motor_data, 2, false);

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