#include "sensor_interface.h"
#include "encoder_config.h"
#include "hiwonder_driver.h"
#include "pico/stdlib.h"
#include <math.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include "tof.h"

static EncoderState encoder_state;
static bool encoder_ready;
static float actual_speed = NAN;
static const EncoderWheelConfig wheels[2] = {
    {ENCODER_LEFT_CHANNEL, ENCODER_LEFT_FORWARD_SIGN,
     ENCODER_LEFT_COUNTS_PER_WHEEL_REV, ENCODER_LEFT_CIRCUMFERENCE_M},
    {ENCODER_RIGHT_CHANNEL, ENCODER_RIGHT_FORWARD_SIGN,
     ENCODER_RIGHT_COUNTS_PER_WHEEL_REV, ENCODER_RIGHT_CIRCUMFERENCE_M}
};

#define TOF_ADDRESS 0x29
#define TOF_I2C_BUS i2c1
#define TOF_SDA_PIN 6
#define TOF_SCL_PIN 7
static bool tof_ready;

// static bool tof_write_register(uint8_t reg, uint8_t value) {
//     uint8_t data[2] = {reg, value};

//     return i2c_write_blocking(
//         TOF_I2C_BUS,
//         TOF_ADDRESS,
//         data,
//         2,
//         false
//     ) == 2;
// }

// static bool tof_read_register(uint8_t reg, uint8_t *value) {
//     if (i2c_write_blocking(
//             TOF_I2C_BUS,
//             TOF_ADDRESS,
//             &reg,
//             1,
//             true
//         ) < 0) {
//         return false;
//     }

//     return i2c_read_blocking(
//         TOF_I2C_BUS,
//         TOF_ADDRESS,
//         value,
//         1,
//         false
//     ) == 1;
// }

// static bool tof_init(void) {
//     uint8_t model_id = 0;

//     i2c_init(TOF_I2C_BUS, 400 * 1000);

//     gpio_set_function(TOF_SDA_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(TOF_SCL_PIN, GPIO_FUNC_I2C);

//     gpio_pull_up(TOF_SDA_PIN);
//     gpio_pull_up(TOF_SCL_PIN);

//     sleep_ms(20);

//     if (!tof_read_register(0xC0, &model_id)) {
//         printf("ToF: no I2C response at 0x29\n");
//         return false;
//     }

//     printf("ToF model ID: 0x%02X, expected 0xEE\n",
//         (unsigned int)model_id);

//     return model_id == 0xEE;
// }

// static bool tof_read_distance_mm(uint16_t *distance_mm) {
//     uint8_t status = 0;
//     uint8_t range_data[2] = {0, 0};
//     bool measurement_started = false;
//     bool measurement_ready = false;

//     if (!tof_write_register(0x00, 0x01)) {
//         return false;
//     }

//     for (int i = 0; i < 100; i++) {
//         uint8_t start = 0;

//         if (!tof_read_register(0x00, &start)) {
//             return false;
//         }

//         if ((start & 0x01) == 0) {
//             measurement_started = true;
//             break;
//         }

//         sleep_ms(1);
//     }

//     if (!measurement_started) {
//         return false;
//     }

//     for (int i = 0; i < 100; i++) {
//         if (!tof_read_register(0x13, &status)) {
//             return false;
//         }

//         if (status & 0x07) {
//             measurement_ready = true;
//             break;
//         }

//         sleep_ms(1);
//     }

//     if (!measurement_ready) {
//         return false;
//     }

//     uint8_t range_register = 0x1E;

//     if (i2c_write_blocking(
//             TOF_I2C_BUS,
//             TOF_ADDRESS,
//             &range_register,
//             1,
//             true
//         ) < 0) {
//         return false;
//     }

//     if (i2c_read_blocking(
//             TOF_I2C_BUS,
//             TOF_ADDRESS,
//             range_data,
//             2,
//             false
//         ) != 2) {
//         return false;
//     }

//     *distance_mm = ((uint16_t)range_data[0] << 8) | range_data[1];

//     tof_write_register(0x0B, 0x01);

//     return true;
// }

static bool tof_init(void) {
    i2c_init(TOF_I2C_BUS, 100000);

    gpio_set_function(TOF_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_SDA_PIN);
    gpio_pull_up(TOF_SCL_PIN);

    sleep_ms(50);

    // Driver performs sensor configuration and reference calibration.
    return tofInit(1, TOF_ADDRESS, 1) != 0;
}

// Configure only the MCU bus, not the board's motor mode or encoder polarity.
void sensors_init(void) {
    encoder_feedback_reset(&encoder_state);
    actual_speed = NAN;
    encoder_ready = ENCODER_ENABLED &&
        hiwonder_init(ENCODER_I2C_BUS, ENCODER_SDA_PIN, ENCODER_SCL_PIN,
                      ENCODER_I2C_BAUDRATE, ENCODER_TRANSFER_TIMEOUT_US);
    tof_ready = tof_init();
    printf("ToF initialization: %s\n", tof_ready ? "OK" : "FAILED");
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
// float sensors_get_radar_distance(void) {
//     uint16_t distance_mm = 0;

//     if (!tof_ready || !tof_read_distance_mm(&distance_mm)) {
//         return NAN;
//     }

//     return (float)distance_mm / 1000.0f;
// }

float sensors_get_radar_distance(void) {
    if (!tof_ready) {
        return NAN;
    }

    int distance_mm = tofReadDistance();
    if (distance_mm <= 0 || distance_mm >= 8190) {
        printf("ToF rejected raw distance: %d mm\n", distance_mm);
    }

    // Includes the driver's timeout sentinel of 65535.
    if (distance_mm <= 0 || distance_mm >= 8190) {
        return NAN;
    }

    return (float)distance_mm / 1000.0f;
}

// Compatibility accessor for future control; NAN propagates missing calibration.
float sensors_get_actual_speed(void) {
    return actual_speed;
}