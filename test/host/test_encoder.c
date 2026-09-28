#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "encoder_feedback.h"
#include "hiwonder_driver.h"
#include "sensor_interface.h"

i2c_inst_t test_i2c0 = {0}, test_i2c1 = {1};
static int write_result = 1, read_result = 16;
static unsigned int reads, writes;
static uint8_t reply[16];
static uint8_t sent[17];
static size_t sent_length;
static bool sent_nostop;
static uint64_t test_time;

// Fake monotonic clock controlled by each sensor-integration test step.
uint64_t time_us_64(void) { return test_time; }

// SDK mock: verify the expected bus without touching hardware.
unsigned int i2c_init(i2c_inst_t *bus, unsigned int baud) {
    assert(bus == i2c0);
    return baud;
}
// SDK mock: ensure initialization selects GPIO4/5 for I2C.
void gpio_set_function(unsigned int pin, unsigned int function) {
    assert(pin == 4 || pin == 5);
    assert(function == 3);
}
// Capture outgoing bytes/STOP behavior and return an injected transfer result.
int i2c_write_timeout_us(i2c_inst_t *bus, uint8_t addr, const uint8_t *data,
                        size_t length, bool nostop, unsigned int timeout) {
    assert(bus == i2c0 && addr == 0x34 && timeout == 3000);
    ++writes;
    memcpy(sent, data, length);
    sent_length = length;
    sent_nostop = nostop;
    return write_result;
}
// Supply synthetic board bytes, including injected short reads and failures.
int i2c_read_timeout_us(i2c_inst_t *bus, uint8_t addr, uint8_t *data,
                       size_t length, bool nostop, unsigned int timeout) {
    assert(bus == i2c0 && addr == 0x34 && timeout == 3000);
    assert(!nostop && length == 16);
    ++reads;
    memcpy(data, reply, length);
    return read_result;
}

// Verify signed wire values, framing, and preservation of output on short/error reads.
static void test_driver(void) {
    int32_t counts[4] = {9, 9, 9, 9};
    assert(!hiwonder_read_encoder_counts(counts));
    assert(!hiwonder_init(i2c0, 2, 3, 100000, 3000));
    assert(hiwonder_init(i2c0, 4, 5, 100000, 3000));
    assert(writes == 0);
    uint8_t payload[16] = {0x78,0x56,0x34,0x12, 0xff,0xff,0xff,0xff,
                          0,0,0,0x80, 0xff,0xff,0xff,0x7f};
    memcpy(reply, payload, 16);
    assert(hiwonder_read_encoder_counts(counts));
    assert(sent_length == 1 && sent[0] == 60 && sent_nostop);
    assert(counts[0] == 0x12345678 && counts[1] == -1);
    assert(counts[2] == INT32_MIN && counts[3] == INT32_MAX);
    int32_t saved[4];
    memcpy(saved, counts, sizeof(saved));
    read_result = 8;
    assert(!hiwonder_read_encoder_counts(counts));
    assert(memcmp(saved, counts, sizeof(saved)) == 0);
    write_result = -1;
    unsigned int old_reads = reads;
    assert(!hiwonder_read_encoder_counts(counts));
    assert(reads == old_reads && memcmp(saved, counts, sizeof(saved)) == 0);
}

// Test calibration is synthetic, never a claimed Hiwonder motor specification.
static void test_feedback(void) {
    EncoderState state = {0};
    EncoderWheelConfig wheels[2] = {{0, 1, 100, 0.5}, {2, -1, 100, 0.5}};
    EncoderFeedback f;
    int32_t counts[4] = {-5, 999, 5, 999};
    assert(!encoder_feedback_update(&state, wheels, counts, 0, &f));
    assert(f.left.raw_count == -5 && f.right.raw_count == 5 && f.dt_s == 0);
    assert(isnan(f.left.speed_mps));
    counts[0] = 5; counts[2] = -5;
    assert(encoder_feedback_update(&state, wheels, counts, 20000, &f));
    assert(f.left.delta_count == 10 && f.right.delta_count == -10);
    assert(fabs(f.dt_s - 0.02) < 1e-9);
    assert(fabs(f.left.speed_mps - 2.5) < 1e-9);
    assert(fabs(f.right.distance_change_m - 0.05) < 1e-9);
    counts[0] = -5; counts[2] = 5;
    assert(encoder_feedback_update(&state, wheels, counts, 60000, &f));
    assert(fabs(f.left.speed_mps + 1.25) < 1e-9);
    assert(fabs(f.left.distance_change_m + 0.05) < 1e-9); // Not accumulated.

    // Bad timing rejects this read; next good read is baseline-only.
    assert(!encoder_feedback_update(&state, wheels, counts, 60000, &f));
    assert(!state.have_previous && isnan(f.left.speed_mps));
    counts[0] = 100;
    assert(!encoder_feedback_update(&state, wheels, counts, 80000, &f));
    assert(encoder_feedback_update(&state, wheels, counts, 100000, &f));
    assert(f.left.delta_count == 0 && f.left.speed_mps == 0);
    assert(!encoder_feedback_update(&state, wheels, counts, 90000, &f));
    assert(!encoder_feedback_update(&state, wheels, counts, 120000, &f));

    // Unknown physical scale does not hide raw deltas; each wheel is independent.
    wheels[0].counts_per_wheel_revolution = 0;
    counts[0] += 2; counts[2] += 2;
    assert(encoder_feedback_update(&state, wheels, counts, 140000, &f));
    assert(f.left.delta_count == 2 && isnan(f.left.speed_mps));
    assert(isfinite(f.right.speed_mps));
    wheels[1].circumference_m = NAN;
    assert(encoder_feedback_update(&state, wheels, counts, 160000, &f));
    assert(isnan(f.right.speed_mps));

    // Widened signed subtraction has no C overflow, and does NOT invent a wrap.
    encoder_feedback_reset(&state);
    counts[0] = INT32_MIN;
    assert(!encoder_feedback_update(&state, wheels, counts, 0, &f));
    counts[0] = INT32_MAX;
    assert(encoder_feedback_update(&state, wheels, counts, 20000, &f));
    assert(f.left.delta_count == INT64_C(4294967295));
    wheels[0].channel = 4;
    assert(!encoder_feedback_update(&state, wheels, counts, 40000, &f));
    assert(!state.have_previous);
}

// Exercise read failure/recovery through the real sensor caller and mocked I2C.
static void test_sensor_integration(void) {
    write_result = 1; read_result = 16; test_time = 0;
    memset(reply, 0, sizeof(reply));
    sensors_init();
    EncoderFeedback f;
    assert(!sensors_update(&f)); // First successful read saves the baseline.
    assert(isnan(sensors_get_actual_speed()));
    test_time = 20000; reply[0] = 10; reply[8] = 20;
    assert(sensors_update(&f));
    assert(f.left.raw_count == 10 && f.right.raw_count == 20);
    assert(f.left.delta_count == 10 && f.right.delta_count == 20);
    assert(isnan(sensors_get_actual_speed())); // Real config remains uncalibrated.
    read_result = -1; test_time = 40000;
    assert(!sensors_update(&f) && isnan(f.left.speed_mps) && f.dt_s == 0);
    read_result = 16; test_time = 60000; reply[0] = 100;
    assert(!sensors_update(&f)); // Recovery must not bridge the failed interval.
    assert(f.left.raw_count == 100 && f.left.delta_count == 0);
    test_time = 80000; reply[0] = 103;
    assert(sensors_update(&f) && f.left.delta_count == 3);
}

// Run the host checks; assertion failures stop execution.
int main(void) {
    test_driver();
    test_feedback();
    test_sensor_integration();
    puts("Minimal encoder driver, feedback, and sensor tests passed.");
    return 0;
}
