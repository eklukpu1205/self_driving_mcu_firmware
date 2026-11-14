#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "spi_protocol.h"

// Jetson SPI0 master (header pins 19/21/23/24) -> MCU SPI1 slave.
// GP12=RX (Jetson MOSI); GP11=TX (Jetson MISO). Common GND required.
#define COMMS_CS   13
#define COMMS_SCLK 10
#define COMMS_MOSI 12
#define COMMS_MISO 11

// ISR receives raw bytes into a software ring. No parsing/printf in ISR.
void comms_spi_init(void);
bool comms_spi_read_byte(uint8_t *byte_out);
uint32_t comms_spi_rx_byte_count(void);
uint32_t comms_spi_hw_overrun_count(void);
uint32_t comms_spi_ring_overrun_count(void);