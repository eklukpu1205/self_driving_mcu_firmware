#ifndef ACTUATOR_HAL_H
#define ACTUATOR_HAL_H

#include <stdint.h>
#include <stdbool.h>


// Max Steering Limits
#define PULSE_ABSOLUTE_MIN_US 500
#define PULSE_NEUTRAL_US 1500
#define PULSE_ABSOLUTE_MAX_US 2500

// Safe Steering Limits (Radians)
#define STEERING_MAX_RAD 0.45f
#define STEERING_MIN_RAD -0.45f

// Safe Steering Pulse Limits (Calculated for +/- 0.45 rad)
#define PULSE_SAFE_LEFT_US 1214
#define PULSE_SAFE_RIGHT_US 1786

// Motor ESC Limits 
#define MOTOR_PULSE_MIN_US 1000
#define MOTOR_PULSE_MAX_US 2000



void actuator_hal_init(void);

void actuator_hal_set_steering_rad(float angle_rad);

void actuator_hal_set_throttle_effort(float throttle_percent);

void actuator_hal_emergency_halt(void);

#endif 


