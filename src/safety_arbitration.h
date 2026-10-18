#include "sensor_interface.h"
#include "actuator_hal.h"
#include <stdbool.h>

bool safety_check(void);
// Safety distance in meters
#define SAFETY_DISTANCE 1