#ifndef ACTUATOR_HAL_H
#define ACTUATOR_HAL_H

#include <stdint.h>
#include <stdbool.h>


// Max Steering Limits
#define PULSE_ABSOLUTE_MIN_US 700
#define PULSE_NEUTRAL_US 1500
#define PULSE_ABSOLUTE_MAX_US 2500






void actuator_hal_init(void);
static void move_steering(int*, int);
void i2c_comm_setup(void);
void set_raw_pulse(int pin, uint16_t pulse_width_us);
void actuator_hal_set_steering_pulse_us(uint16_t pulse_us); 
// Servo command: 0..180 degrees, with 90 at neutral. Not wheel angle.
// Out-of-range values are clamped. NaN/infinity return false without moving.
bool actuator_hal_set_steering_angle_deg(float angle_deg);
bool actuator_hal_set_throttle_effort(float throttle_percent);
void actuator_hal_emergency_halt(void);


#endif
