#include "sensor_interface.h"
#include "encoder_config.h"
#include "hiwonder_driver.h"
#include "pico/stdlib.h"
#include <math.h>

static EncoderState encoder_state;
static bool encoder_ready;
static float actual_speed = NAN;
static const EncoderWheelConfig wheels[2] = {
    {ENCODER_LEFT_CHANNEL, ENCODER_LEFT_FORWARD_SIGN,
     ENCODER_LEFT_COUNTS_PER_WHEEL_REV, ENCODER_LEFT_CIRCUMFERENCE_M},
    {ENCODER_RIGHT_CHANNEL, ENCODER_RIGHT_FORWARD_SIGN,
     ENCODER_RIGHT_COUNTS_PER_WHEEL_REV, ENCODER_RIGHT_CIRCUMFERENCE_M}
};

// Configure only the MCU bus, not the board's motor mode or encoder polarity.
void sensors_init(void) {
    encoder_feedback_reset(&encoder_state);
    actual_speed = NAN;
    encoder_ready = ENCODER_ENABLED &&
        hiwonder_init(ENCODER_I2C_BUS, ENCODER_SDA_PIN, ENCODER_SCL_PIN,
                      ENCODER_I2C_BAUDRATE, ENCODER_TRANSFER_TIMEOUT_US);
}

// Read all totals, timestamp reception, then calculate the selected wheel deltas.
bool sensors_update(EncoderFeedback *out) {
    int32_t counts[HIWONDER_CHANNELS];
    actual_speed = NAN;
    *out = (EncoderFeedback){
        .left = {.distance_change_m = NAN, .speed_mps = NAN},
        .right = {.distance_change_m = NAN, .speed_mps = NAN}
    };
    if (!encoder_ready || !hiwonder_read_encoder_counts(counts)) {
        encoder_feedback_reset(&encoder_state);
        return false;
    }
    bool have_delta = encoder_feedback_update(&encoder_state, wheels, counts, time_us_64(), out);
    if (have_delta) {
        actual_speed = (float)((out->left.speed_mps + out->right.speed_mps) * 0.5);
    }
    return have_delta;
}

// Existing teammate-owned radar placeholder.
float sensors_get_radar_distance(void) {
    // Get radar sensor data
    return 0.0f;
}

// Compatibility accessor for future control; NAN propagates missing calibration.
float sensors_get_actual_speed(void) {
    return actual_speed;
}
