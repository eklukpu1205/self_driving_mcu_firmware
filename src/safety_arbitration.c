#include "safety_arbitration.h"
#include "actuator_hal.h"
#include <math.h> // had to add this (eitan)

bool safety_check (void){

    float object_distance = sensors_get_radar_distance();

    if ((!isfinite(object_distance)) || (object_distance <= SAFETY_DISTANCE)){
        actuator_hal_emergency_halt();
        return false;
    }

    return true;
}