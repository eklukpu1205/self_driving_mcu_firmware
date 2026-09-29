#include "safety_arbitration.h"
#include "actuator_hal.h"

void safety_check (void){

    float object_distance = sensors_get_radar_distance();

    if (object_distance <= SAFETY_DISTANCE){
        actuator_hal_emergency_halt();
        bool system_safe = false;
    }

    return;
}