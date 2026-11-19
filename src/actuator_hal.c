#include "actuator_hal.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h" // Eitan: for sleep_ms
#include <math.h>

// Pin Definitions
#define PIN_SERVO_PWM 14

#define MOTOR_I2C_SDA_PIN 4
#define MOTOR_I2C_SCL_PIN 5
#define MOTOR_PWM_ADDR 0x1F

// PWM Timing Calculations (50 Hz)
#define PWM_WRAP_50HZ 19999

static uint servo_slice;

void actuator_hal_init(void) {
  // Assign pins to the PWM peripheral function
  gpio_set_function(PIN_SERVO_PWM, GPIO_FUNC_PWM);

  // Identify the hardware associated with each GPIO
  servo_slice = pwm_gpio_to_slice_num(PIN_SERVO_PWM);

  // Configure the PWM
  pwm_config config = pwm_get_default_config();

  uint32_t sys_clk = clock_get_hz(clk_sys);
  float clk_div = (float)sys_clk / 1000000.0f;

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

  // Eitan: set motor type to 0. With types 1-3 the motors only jitter.
  // Try for about 2 s in case the motor board is still starting up.
  uint8_t type_cmd[2] = {MOTOR_TYPE_ADDR, 0};
  for (int i = 0; i < 10; i++) {
    if (i2c_write_timeout_us(i2c0, HI_WONDER_I2C_ADDR, type_cmd, 2, false,
                             3000) == 2) {
      break;
    }
    sleep_ms(200);
  }

  // Ensure motor throttle starts explicitly zeroed and servo neutral
  actuator_hal_emergency_halt();
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

bool actuator_hal_set_steering_angle_deg(float angle_deg) {
  if (!isfinite(angle_deg)) {
    return false;
  }

  if (angle_deg < 0.0f)
    angle_deg = 0.0f;
  if (angle_deg > 180.0f)
    angle_deg = 180.0f;

  // Use the existing pulse limits
  float pulse_us;
  if (angle_deg <= 90.0f) {
    pulse_us = PULSE_ABSOLUTE_MIN_US +
               (angle_deg / 90.0f) * (PULSE_NEUTRAL_US - PULSE_ABSOLUTE_MIN_US);
  } else {
    pulse_us =
        PULSE_NEUTRAL_US + ((angle_deg - 90.0f) / 90.0f) *
                               (PULSE_ABSOLUTE_MAX_US - PULSE_NEUTRAL_US);
  }

  actuator_hal_set_steering_pulse_us((uint16_t)lroundf(pulse_us));
  return true;
}

bool actuator_hal_set_throttle_effort(float throttle_percent) {
  if (!isfinite(throttle_percent)) {
    return false; // had to add this (eitan)
  }

  // Utilize I2C to adjust the motor speed
  if (throttle_percent < -100.0f) {
    throttle_percent = -100.0f;
  }
  if (throttle_percent > 100.0f) {
    throttle_percent = 100.0f;
  }

  int8_t pwm_val = (int8_t)throttle_percent;

  uint8_t motor_data[5] = {MOTOR_PWM_ADDR, (uint8_t)(-pwm_val), 0,
                           (uint8_t)(pwm_val), 0};

  int result = i2c_write_timeout_us(i2c0, HI_WONDER_I2C_ADDR, motor_data,
                                    sizeof(motor_data), false, 3000);

  return result == (int)sizeof(motor_data);
}

void actuator_hal_emergency_halt(void) {
  // Sets the steering to neutral state
  set_raw_pulse(PIN_SERVO_PWM, PULSE_NEUTRAL_US);

  // Add control signal to set the motor speed to neutral
  actuator_hal_set_throttle_effort(0);
}

void i2c_comm_setup(void) {
  // Initializes the I2C communication
  i2c_init(i2c0, 100000);

  gpio_set_function(MOTOR_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_set_function(MOTOR_I2C_SDA_PIN, GPIO_FUNC_I2C);

  gpio_pull_up(MOTOR_I2C_SCL_PIN);
  gpio_pull_up(MOTOR_I2C_SDA_PIN);
}