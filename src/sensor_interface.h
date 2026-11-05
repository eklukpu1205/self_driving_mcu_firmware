#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "encoder_feedback.h"

void sensors_init(void);
bool sensors_update(EncoderFeedback *out);
float sensors_get_radar_distance(void);
float sensors_get_actual_speed(void);

// Non-blocking ToF interfaces
bool sensors_trigger_tof(void);
bool sensors_is_tof_ready(void);
bool sensors_fetch_tof(void);