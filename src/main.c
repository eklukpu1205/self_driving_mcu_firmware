#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

// Include subsystem headers
#include "comms_spi.h"
#include "safety_arbitration.h"
#include "sensor_interface.h"
#include "actuator_hal.h"
#include "encoder_feedback.h"
#include "hiwonder_driver.h"
#include "spi_protocol.h"

#define LOOP_PERIOD_US 20000 // 50 Hz
#define COMMAND_TIMEOUT_US 250000u

// Flag set by the 50 Hz hardware timer interrupt
static volatile bool tick_50hz = false;

// 50 Hz Hardware Timer Interrupt Callback
static bool timer_50hz_callback(struct repeating_timer *t) {
    (void)t;
    tick_50hz = true;
    return true; // Keep repeating every 20 ms
}

int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    actuator_hal_init();

    sleep_ms(3000);

    sensors_init();

    // Initializes SPI1 and enables SPI1_IRQ background handler
    comms_spi_init(); 

    // Kick off the very first ToF measurement
    sensors_trigger_tof();

    // Start the 50 Hz hardware timer interrupt
    struct repeating_timer timer;
    add_repeating_timer_us(-LOOP_PERIOD_US, timer_50hz_callback, NULL, &timer);

    uint8_t frame[SPI_CMD_BYTES];
    unsigned used = 0;
    SpiCommand latest_cmd = {0};
    bool have_command = false;
    bool command_timeout = false;

    uint64_t now;
    uint64_t last_valid_us = time_us_64();

    EncoderFeedback wheel_feedback = {0}; 

    while (true) {
       
        // If the ToF GPIO interrupt fired, fetch the data and start the next reading immediately
        if (sensors_is_tof_ready()) {
            sensors_fetch_tof();
            sensors_trigger_tof(); 
        }

        // Read bytes from the ring buffer populated by SPI1_IRQ
        uint8_t b;
        while (comms_spi_read_byte(&b)) {
            frame[used++] = b;
            
            if (used == SPI_CMD_BYTES) {
                SpiCommand cmd;
                // Directly decode the fixed-size packet 
                if (spi_command_decode(frame, &cmd)) {
                    latest_cmd = cmd;
                    have_command = true;
                    last_valid_us = time_us_64();
                    command_timeout = false;
                }
                used = 0; // Reset frame index for the next packet
            }
        }

        // Check command safety timeout (250 ms without valid packet)
        now = time_us_64();
        if (!command_timeout && (now - last_valid_us > COMMAND_TIMEOUT_US)) {
            command_timeout = true;
            have_command = false; 
            actuator_hal_emergency_halt();
        }

        // 50 Hz MCU control scheduler triggered precisely by the timer interrupt
        if (tick_50hz) {
            tick_50hz = false;

            // Check radar / safety constraints (evaluates ToF distance internally)
            bool system_safe = !safety_check(); 

            // Convert protocol units to actuators
            float target_speed = (float)latest_cmd.target_speed_mm_s / 1000.0f;
            float target_steering = (float)latest_cmd.target_steering_mrad;

            // Execute movement commands if system is safe and valid data is present
            if (system_safe && have_command && !latest_cmd.emergency_brake && !command_timeout) {
                actuator_set_speed(target_speed);
                actuator_set_steering(target_steering);
            } else {
                actuator_hal_emergency_halt(); 
            }

            // Read board-processed encoder telemetry updates
            sensors_update(&wheel_feedback);
        }

        // Put CPU to sleep until the next interrupt 
        tight_loop_contents();
    }
    return 0;
}