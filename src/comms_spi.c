#include "comms_spi.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/regs/spi.h"

#define RX_RING_SIZE 512u // Power of two, single IRQ producer + main-loop consumer.

static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_errors;

static void spi1_rx_irq(void) {
    spi_hw_t *hw = spi_get_hw(spi1);

    if (hw->ris & SPI_SSPRIS_RORRIS_BITS) {
        ++rx_errors; // Hardware RX FIFO overflow: at least one byte was lost.
    }

    while (spi_is_readable(spi1)) {
        uint8_t byte = (uint8_t)hw->dr;
        uint16_t next = (rx_head + 1u) & (RX_RING_SIZE - 1u);
        if (next != rx_tail) {
            rx_ring[rx_head] = byte;
            rx_head = next;
        } else {
            ++rx_errors; // Software ring overrun.
        }
    }

    // Clear the SPI RX timeout and overrun interrupt latches.
    hw->icr = SPI_SSPICR_RTIC_BITS | SPI_SSPICR_RORIC_BITS;
}

void comms_spi_init(void) {
    rx_head = rx_tail = 0;
    rx_errors = 0;

    spi_init(spi1, 1000000); // Slave: Jetson controls the actual clock.
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(spi1, true);

    gpio_set_function(COMMS_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_MISO, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_CS, GPIO_FUNC_SPI);

    spi_hw_t *hw = spi_get_hw(spi1);
    while (spi_is_readable(spi1)) (void)hw->dr;
    hw->icr = SPI_SSPICR_RTIC_BITS | SPI_SSPICR_RORIC_BITS;

    irq_set_exclusive_handler(SPI1_IRQ, spi1_rx_irq);
    hw->imsc = SPI_SSPIMSC_RXIM_BITS | SPI_SSPIMSC_RTIM_BITS |
               SPI_SSPIMSC_RORIM_BITS;
    irq_set_enabled(SPI1_IRQ, true);
}

bool comms_spi_read_byte(uint8_t *out) {
    if (out == NULL || rx_tail == rx_head) return false;
    *out = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1u) & (RX_RING_SIZE - 1u);
    return true;
}

uint32_t comms_spi_error_count(void) {
    return rx_errors;
}
