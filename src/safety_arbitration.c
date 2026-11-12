// safety_arbitration.c
#include "safety_arbitration.h"
#include "sensor_interface.h"
#include "actuator_hal.h"
#include <math.h>

#define EMERGENCY_STOP_DISTANCE_M 0.50f

#include "pico/stdlib.h"

#define OBSTACLE_HOLD_US 1000000u   // stay stopped 1 s after any obstacle reading

bool safety_check(void) {
    static uint64_t hold_until_us = 0;
    float distance_m = sensors_get_radar_distance();
    uint64_t now = time_us_64();

    if (!isnan(distance_m) && distance_m < EMERGENCY_STOP_DISTANCE_M) {
        hold_until_us = now + OBSTACLE_HOLD_US;      // obstacle seen: start hold
    }
    if (isnan(distance_m) || now < hold_until_us) {
        actuator_hal_emergency_halt();
        return true;
    }
    return false;
}