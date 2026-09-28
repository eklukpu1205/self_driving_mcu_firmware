#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/i2c.h"

#define HIWONDER_ADDRESS 0x34
#define HIWONDER_CHANNELS 4

// Single main-loop owner: do not call from IRQs or a second core.
// Initialization configures the host only; it does not command motors.
bool hiwonder_init(i2c_inst_t *bus, unsigned int sda, unsigned int scl,
                   uint32_t baudrate, uint32_t timeout_us);
// Read 1..16 bytes; false on invalid arguments or I2C failure. Output unchanged.
bool hiwonder_read_registers(uint8_t reg, uint8_t *data, size_t length);
// Four signed little-endian int32 totals in M1..M4 order (see protocol sources).
// Output unchanged on failure; no A/B edge decoding on the MCU.
bool hiwonder_read_encoder_counts(int32_t counts[HIWONDER_CHANNELS]);
