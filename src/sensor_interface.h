#pragma once
#include "encoder_feedback.h"

// Initialize the host I2C interface and forget previous encoder counts.
void sensors_init(void);
// Read the board into non-null out. True means deltas available.
// On failure output is cleared (metric values NAN), and the baseline is reset.
// On a first good read output contains raw counts but return is false.
bool sensors_update(EncoderFeedback *out);
// Existing radar placeholder; zero is not a measured distance.
float sensors_get_radar_distance(void);
// Mean speed from the most recent update, or NAN. No independent bus read/expiry.
float sensors_get_actual_speed(void);