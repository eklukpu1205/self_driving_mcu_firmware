#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    unsigned int channel; // M1=0, M2=1, M3=2, M4=3.
    int forward_sign;     // Start with +1; change to -1 only after bench verification.
    double counts_per_wheel_revolution;
    double circumference_m;
} EncoderWheelConfig;

// The only history needed: previous selected counts and their reception time.
typedef struct {
    int32_t previous_count[2];
    uint64_t previous_time_us;
    bool have_previous;
} EncoderState;

typedef struct {
    int32_t raw_count;
    int64_t delta_count; // Uncorrected current - previous for bench inspection.
    double distance_change_m; // This interval only, with forward_sign applied.
    double speed_mps;
} WheelFeedback;

typedef struct {
    WheelFeedback left;
    WheelFeedback right;
    double dt_s; // Zero when no delta is available.
} EncoderFeedback;

// Forget the previous sample after a read failure or known board reset.
void encoder_feedback_reset(EncoderState *state);
// Hiwonder already decodes A/B. Consume its four signed total counts here.
// True means raw deltas and dt are available, even without physical calibration.
// All pointer arguments must be non-null. False means baseline-only, unusable
// timing or invalid channel selection. First/recovery
// samples expose raw counts but no delta. Unknown distance/speed are NAN.
bool encoder_feedback_update(EncoderState *state, const EncoderWheelConfig wheels[2],
                             const int32_t counts[4], uint64_t now_us,
                             EncoderFeedback *out);
