#include "encoder_feedback.h"
#include <math.h>

// Keep reset explicit so a failed bus read cannot be bridged by the next delta.
void encoder_feedback_reset(EncoderState *state) {
    *state = (EncoderState){0};
}

// Calculate a single interval; no hardware access, cached snapshot or distance sum.
bool encoder_feedback_update(EncoderState *state, const EncoderWheelConfig wheels[2],
                             const int32_t counts[4], uint64_t now_us,
                             EncoderFeedback *out) {
    *out = (EncoderFeedback){
        .left = {.distance_change_m = NAN, .speed_mps = NAN},
        .right = {.distance_change_m = NAN, .speed_mps = NAN}
    };
    if (wheels[0].channel >= 4 || wheels[1].channel >= 4 ||
        wheels[0].channel == wheels[1].channel) {
        encoder_feedback_reset(state);
        return false;
    }
    WheelFeedback *result[2] = {&out->left, &out->right};
    for (unsigned int i = 0; i < 2; ++i) {
        result[i]->raw_count = counts[wheels[i].channel];
    }
    if (state->have_previous && now_us <= state->previous_time_us) {
        // Reject this timestamp; the NEXT good sample establishes a baseline.
        encoder_feedback_reset(state);
        return false;
    }
    bool have_delta = state->have_previous;
    if (have_delta) {
        out->dt_s = (double)(now_us - state->previous_time_us) / 1000000.0;
    }
    for (unsigned int i = 0; i < 2; ++i) {
        WheelFeedback *wheel = result[i];
        if (have_delta) {
            // Widen before subtraction to avoid signed C overflow. No wrap correction.
            wheel->delta_count = (int64_t)wheel->raw_count - state->previous_count[i];
            double cpr = wheels[i].counts_per_wheel_revolution;
            double circumference = wheels[i].circumference_m;
            if (isfinite(cpr) && cpr > 0 && isfinite(circumference) && circumference > 0 &&
                (wheels[i].forward_sign == 1 || wheels[i].forward_sign == -1)) {
                wheel->distance_change_m = wheels[i].forward_sign *
                    (double)wheel->delta_count / cpr * circumference;
                wheel->speed_mps = wheel->distance_change_m / out->dt_s;
            }
        }
        state->previous_count[i] = wheel->raw_count;
    }
    state->previous_time_us = now_us;
    state->have_previous = true;
    return have_delta;
}
