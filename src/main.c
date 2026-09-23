#include "pico/stdlib.h"
#include "hardware/spi.h"

// Include subsystem headers
#include "comms_spi.h"
#include "safety_arbitration.h"
#include "sensor_interface.h"
#include "actuator_hal.h"
#include "control_loops.h"

#define LOOP_PERIOD_US 20000 // 50 Hz

int main() {
    // Initialize USB/UART for standard printf debugging
    stdio_init_all();
    
    // Initialize all hardware peripherals and software states
    comms_spi_init();
    actuator_hal_init();

    sleep_ms(3000);

    sensors_init();
    control_pid_init(); 

    // Data structures for SPI exchange
    NvidiaCommand incoming_cmd = {0};
    McuCommand outgoing_cmd = {0}; 

    while (1) {
        // Mark the start time of the loop
        uint64_t loop_start = time_us_64();

        // Check if SPI is readable, then call comms_spi_exchange()

        // Read local hardware sensors, and get radar distance and actual wheel speed

        // Safety check and pass Jetson's target speed and radar distance through safety_arbitrate_speed()

        // Set motor and steering

        // Update outgoing_cmd with the actual speed and steering for the next SPI cycle

        // Sleep until loop cycle has passed
        sleep_until(loop_start + LOOP_PERIOD_US); 
    }
    return 0;
}