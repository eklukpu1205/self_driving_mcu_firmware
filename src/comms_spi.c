#include "comms_spi.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/regs/spi.h"

// Single-core Stage 1: SPI IRQ is producer; main loop is consumer.
// Ring holds up to 511 bytes (one slot unused to distinguish full/empty).
#define RX_RING_SIZE 512u
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_bytes;
static volatile uint32_t hw_overruns;
static volatile uint32_t ring_overruns;

static void spi1_irq_handler(void) {
    spi_hw_t *hw = spi_get_hw(spi1);

    if (hw->ris & SPI_SSPRIS_RORRIS_BITS) {
        ++hw_overruns;
    }

    // Read FIFO promptly; this IRQ is also triggered by RX timeout for
    // trailing bytes that didn't reach the FIFO interrupt threshold.
    while (spi_is_readable(spi1)) {
        uint8_t b = (uint8_t)hw->dr;
        ++rx_bytes;
        uint16_t next = (uint16_t)((rx_head + 1u) & (RX_RING_SIZE - 1u));
        if (next == rx_tail) {
            ++ring_overruns;
        } else {
            rx_ring[rx_head] = b;
            rx_head = next;
        }
    }
    hw->icr = SPI_SSPICR_RTIC_BITS | SPI_SSPICR_RORIC_BITS;
}

void comms_spi_init(void) {
    rx_head = rx_tail = 0;
    rx_bytes = hw_overruns = ring_overruns = 0;
    spi_init(spi1, 1000000); // Slave; Jetson dictates SCK frequency.
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(spi1, true);
    gpio_set_function(COMMS_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_MISO, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_CS, GPIO_FUNC_SPI);

    spi_hw_t *hw = spi_get_hw(spi1);
    while (spi_is_readable(spi1)) (void)hw->dr;
    hw->icr = SPI_SSPICR_RTIC_BITS | SPI_SSPICR_RORIC_BITS;
    irq_set_exclusive_handler(SPI1_IRQ, spi1_irq_handler);
    hw->imsc = SPI_SSPIMSC_RXIM_BITS | SPI_SSPIMSC_RTIM_BITS |
               SPI_SSPIMSC_RORIM_BITS;
    irq_set_enabled(SPI1_IRQ, true);
}

bool comms_spi_read_byte(uint8_t *byte_out) {
    if (byte_out == NULL || rx_tail == rx_head) return false;
    *byte_out = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & (RX_RING_SIZE - 1u));
    return true;
}

uint32_t comms_spi_rx_byte_count(void) { return rx_bytes; }
uint32_t comms_spi_hw_overrun_count(void) { return hw_overruns; }
uint32_t comms_spi_ring_overrun_count(void) { return ring_overruns; }