#include "sensor_interface.h"
#include "encoder_config.h"
#include "hiwonder_driver.h"
#include "pico/stdlib.h"
#include <math.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

static EncoderState encoder_state;
static bool encoder_ready;
static float actual_speed = NAN;
static const EncoderWheelConfig wheels[2] = {
    {ENCODER_LEFT_CHANNEL, ENCODER_LEFT_FORWARD_SIGN,
     ENCODER_LEFT_COUNTS_PER_WHEEL_REV, ENCODER_LEFT_CIRCUMFERENCE_M},
    {ENCODER_RIGHT_CHANNEL, ENCODER_RIGHT_FORWARD_SIGN,
     ENCODER_RIGHT_COUNTS_PER_WHEEL_REV, ENCODER_RIGHT_CIRCUMFERENCE_M}
};

#define TOF_I2C_BUS   i2c0
#define TOF_SDA_PIN   4
#define TOF_SCL_PIN   5
#define TOF_INT_PIN   15 // ToF Interrupt Pin 
#define TOF_ADDRESS   0x29

static bool tof_ready;
static volatile bool tof_data_ready = false;

// GPIO Interrupt Handler for ToF Sensor
static void tof_gpio_isr(uint gpio, uint32_t events) {
    if (gpio == TOF_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        tof_data_ready = true;
    }
}

static bool tof_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};

    return i2c_write_blocking(
        TOF_I2C_BUS,
        TOF_ADDRESS,
        data,
        2,
        false
    ) == 2;
}

static bool tof_read_register(uint8_t reg, uint8_t *value) {
    if (i2c_write_blocking(
            TOF_I2C_BUS,
            TOF_ADDRESS,
            &reg,
            1,
            true
        ) < 0) {
        return false;
    }

    return i2c_read_blocking(
        TOF_I2C_BUS,
        TOF_ADDRESS,
        value,
        1,
        false
    ) == 1;
}

static bool tof_init(void) {
    uint8_t model_id = 0;

    i2c_init(TOF_I2C_BUS, 400 * 1000);

    gpio_set_function(TOF_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(TOF_SDA_PIN);
    gpio_pull_up(TOF_SCL_PIN);

    // Initialize ToF Interrupt Pin
    gpio_init(TOF_INT_PIN);
    gpio_set_dir(TOF_INT_PIN, GPIO_IN);
    gpio_pull_up(TOF_INT_PIN);
    gpio_set_irq_enabled_with_callback(TOF_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &tof_gpio_isr);

    sleep_ms(20);

    if (!tof_read_register(0xC0, &model_id)) {
        return false;
    }

    return model_id == 0xEE;
}

static bool tof_read_distance_mm(uint16_t *distance_mm) {
    uint8_t range_data[2] = {0, 0};

    // Trigger single shot measurement
    if (!tof_write_register(0x00, 0x01)) {
        return false;
    }

    // Reset interrupt flag and wait for the hardware interrupt
    tof_data_ready = false;
    uint32_t start_time = time_us_32();
    while (!tof_data_ready) {
        if (time_us_32() - start_time > 50000u) { // 50ms timeout fallback
            break; 
        }
        tight_loop_contents();
    }

    uint8_t range_register = 0x1E;

    if (i2c_write_blocking(
            TOF_I2C_BUS,
            TOF_ADDRESS,
            &range_register,
            1,
            true
        ) < 0) {
        return false;
    }

    if (i2c_read_blocking(
            TOF_I2C_BUS,
            TOF_ADDRESS,
            range_data,
            2,
            false
        ) != 2) {
        return false;
    }

    *distance_mm = ((uint16_t)range_data[0] << 8) | range_data[1];

    tof_write_register(0x0B, 0x01);

    return true;
}

// Configure only the MCU bus, not the board's motor mode or encoder polarity.
void sensors_init(void) {
    encoder_feedback_reset(&encoder_state);
    actual_speed = NAN;
    encoder_ready = ENCODER_ENABLED &&
        hiwonder_init(ENCODER_I2C_BUS, ENCODER_SDA_PIN, ENCODER_SCL_PIN,
                      ENCODER_I2C_BAUDRATE, ENCODER_TRANSFER_TIMEOUT_US);
    tof_ready = tof_init();
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
    uint16_t distance_mm = 0;

    if (!tof_ready || !tof_read_distance_mm(&distance_mm)) {
        return NAN;
    }

    return (float)distance_mm / 1000.0f;
}

// Compatibility accessor for future control; NAN propagates missing calibration.
float sensors_get_actual_speed(void) {
    return actual_speed;
}