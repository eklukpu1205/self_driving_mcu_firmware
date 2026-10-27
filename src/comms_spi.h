#pragma once
#include <stdbool.h>
#include <stdint.h>

// Jetson SPI0 master -> RP2 SPI1 slave, bench test only.
#define COMMS_CS       13u
#define COMMS_SCLK     10u
#define COMMS_MOSI     12u  // RP SPI1 RX
#define COMMS_MISO     11u  // RP SPI1 TX

#define COMMS_FRAME_BYTES 32u

// Receive-side SPI interrupt and ring buffer: never print inside the IRQ.
void comms_spi_init(void);
bool comms_spi_read_byte(uint8_t *out);
uint32_t comms_spi_error_count(void);