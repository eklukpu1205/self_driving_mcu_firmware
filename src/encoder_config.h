#pragma once

// User-confirmed prototype wiring: SDA=GPIO4, SCL=GPIO5, board address=0x34.
#define ENCODER_ENABLED 1
#define ENCODER_I2C_BUS i2c0
#define ENCODER_SDA_PIN 4u
#define ENCODER_SCL_PIN 5u
// Initial host-side choices, not claimed board specifications.
#define ENCODER_I2C_BAUDRATE 100000u
#define ENCODER_TRANSFER_TIMEOUT_US 3000u

// Provisional left=M1, right=M3; verify by rotating each wheel separately.
#define ENCODER_LEFT_CHANNEL 0u
#define ENCODER_RIGHT_CHANNEL 2u
// +1 leaves direction unchanged; use -1 only if forward rotation counts down.
#define ENCODER_LEFT_FORWARD_SIGN 1
#define ENCODER_RIGHT_FORWARD_SIGN 1
// Measure counts per WHEEL revolution and circumference; zero skips conversion.
#define ENCODER_LEFT_COUNTS_PER_WHEEL_REV 0.0
#define ENCODER_RIGHT_COUNTS_PER_WHEEL_REV 0.0
#define ENCODER_LEFT_CIRCUMFERENCE_M 0.0
#define ENCODER_RIGHT_CIRCUMFERENCE_M 0.0
