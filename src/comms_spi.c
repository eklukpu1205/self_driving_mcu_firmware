#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "comms_spi.h"
#include <string.h>

void comms_spi_init(void) {
 
    spi_init(spi1, 1000000);

 
    spi_set_slave(spi1, true);

    gpio_set_function(COMMS_SCLK, GPIO_FUNC_SPI);  
    gpio_set_function(COMMS_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_MISO, GPIO_FUNC_SPI);
    gpio_set_function(COMMS_CS,   GPIO_FUNC_SPI); 
}

void comms_spi_exchange(NvidiaCommand* cmd_in, const McuCommand* cmd_out) {

    uint8_t rx_buffer[sizeof(NvidiaCommand)] = {0};
    uint8_t tx_buffer[sizeof(McuCommand)] = {0};

    
    memcpy(tx_buffer, cmd_out, sizeof(McuCommand));

    
    spi_write_read_blocking(spi1, tx_buffer, rx_buffer, sizeof(rx_buffer));

   
    memcpy(cmd_in, rx_buffer, sizeof(NvidiaCommand));
}