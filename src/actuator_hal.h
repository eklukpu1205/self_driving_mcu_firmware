#ifndef ACTUATOR_HAL_H
#define ACTUATOR_HAL_H

#include <stdint.h>
#include <stdbool.h>


// Max Steering Limits
#define PULSE_ABSOLUTE_MIN_US 1000
#define PULSE_NEUTRAL_US 1500
#define PULSE_ABSOLUTE_MAX_US 2000






void actuator_hal_init(void);
void i2c_comm_setup(void);
void set_raw_pulse(int pin, uint16_t pulse_width_us);
void actuator_hal_set_steering_pulse_us(uint16_t pulse_us); 
bool actuator_hal_set_throttle_effort(float throttle_percent);
void actuator_hal_emergency_halt(void);

#endif