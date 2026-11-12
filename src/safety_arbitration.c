// safety_arbitration.c
#include "safety_arbitration.h"
#include "sensor_interface.h"
#include "actuator_hal.h"
#include <math.h>

#define EMERGENCY_STOP_DISTANCE_M 0.50f

bool safety_check(void) {
    float distance_m = sensors_get_radar_distance();

    if (!isnan(distance_m) &&
        distance_m > 0.0f &&
        distance_m < EMERGENCY_STOP_DISTANCE_M) {

        actuator_hal_emergency_halt();
        return true;
    }

    return false;
}