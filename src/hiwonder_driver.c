#include "hiwonder_driver.h"
#include "hardware/gpio.h"
#include <string.h>

static i2c_inst_t *driver_bus;
static uint32_t transfer_timeout_us;

// Configure RP2040 I2C and validate pin routing. Returns false for invalid settings.
// timeout_us bounds each transfer separately; no board registers are changed here.
bool hiwonder_init(i2c_inst_t *bus, unsigned int sda, unsigned int scl,
                   uint32_t baudrate, uint32_t timeout_us) {
    driver_bus = NULL;
    if ((bus != i2c0 && bus != i2c1) || sda > 29 || scl > 29 ||
        sda % 2 != 0 || scl % 2 != 1 || baudrate == 0 || timeout_us == 0) {
        return false;
    }
    unsigned int index = bus == i2c0 ? 0 : 1;
    if ((sda / 2) % 2 != index || (scl / 2) % 2 != index) {
        return false;
    }
    // Start the selected peripheral at the requested bus rate (Hz).
    i2c_init(bus, baudrate);
    // Route both GPIOs to the I2C peripheral instead of software GPIO control.
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    // External pull-ups must be suitable for RP2040 3.3 V logic.
    driver_bus = bus;
    transfer_timeout_us = timeout_us;
    return true;
}

// Read 1..16 bytes starting at reg. Preserve the caller buffer on any failure.
bool hiwonder_read_registers(uint8_t reg, uint8_t *data, size_t length) {
    if (!driver_bus || !data || length == 0 || length > 16) return false;
    uint8_t buffer[16];
    // Select the register; nostop=true keeps the bus for a repeated START.
    if (i2c_write_timeout_us(driver_bus, HIWONDER_ADDRESS, &reg, 1, true,
                             transfer_timeout_us) != 1) return false;
    // Receive the full payload and issue STOP; short reads are errors.
    if (i2c_read_timeout_us(driver_bus, HIWONDER_ADDRESS, buffer, length, false,
                            transfer_timeout_us) != (int)length) return false;
    // Publish only after both transfers succeed.
    memcpy(data, buffer, length);
    return true;
}

// Read the board-processed totals, matching the signed format used by PX4.
bool hiwonder_read_encoder_counts(int32_t counts[HIWONDER_CHANNELS]) {
    uint8_t bytes[16];
    // Register 60 exposes all four channels; do not publish partial results.
    if (!counts || !hiwonder_read_registers(60, bytes, sizeof(bytes))) return false;
    for (unsigned int channel = 0; channel < HIWONDER_CHANNELS; ++channel) {
        uint32_t value = 0;
        for (unsigned int byte = 0; byte < 4; ++byte) {
            value |= (uint32_t)bytes[channel * 4 + byte] << (byte * 8);
        }
        // Interpret the wire sign bit explicitly, avoiding an out-of-range cast.
        // This is signed decoding, not counter rollover reconstruction.
        int64_t signed_value = value;
        if (value & UINT32_C(0x80000000)) signed_value -= INT64_C(4294967296);
        counts[channel] = (int32_t)signed_value;
    }
    return true;
}
