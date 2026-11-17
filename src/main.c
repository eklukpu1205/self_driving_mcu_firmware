#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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
    // Allow brief time for USB serial/clocks to stabilize
    sleep_ms(250);

    // Initialize actuators (PWM servo and motor I2C driver) to safe neutral/halt state
    actuator_hal_init();

    // Initialize sensors (encoders enabled, ToF bypassed)
    sensors_init();

    // Initializes SPI1 slave and enables SPI1_IRQ background handler
    comms_spi_init(); 

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
        
        // Read bytes from the ring buffer populated by SPI1_IRQ
        uint8_t b;
        while (comms_spi_read_byte(&b)) {
            frame[used++] = b;
            
            // Framing synchronization state machine:
            // Byte 0 must match low byte of SPI_CMD_MAGIC (0x5A)
            if (used == 1) {
                if (frame[0] != (uint8_t)(SPI_CMD_MAGIC & 0xFF)) {
                    used = 0; // Discard non-marker byte immediately
                }
            } 
            // Byte 1 must match high byte of SPI_CMD_MAGIC (0xA5)
            else if (used == 2) {
                if (frame[1] != (uint8_t)((SPI_CMD_MAGIC >> 8) & 0xFF)) {
                    // If byte 1 happens to be 0x5A, slide it to byte 0
                    if (frame[1] == (uint8_t)(SPI_CMD_MAGIC & 0xFF)) {
                        frame[0] = frame[1];
                        used = 1;
                    } else {
                        used = 0;
                    }
                }
            } 
            // Complete 10-byte packet received: validate and decode
            else if (used == SPI_CMD_BYTES) {
                SpiCommand cmd;
                if (spi_command_decode(frame, &cmd)) {
                    latest_cmd = cmd;
                    have_command = true;
                    last_valid_us = time_us_64();
                    command_timeout = false;
                    used = 0;
                } else {
                    // Frame validation failed (CRC mismatch or out-of-range value).
                    // Slide forward searching for the next 0x5A to regain sync.
                    size_t next_sync = 1;
                    while (next_sync < SPI_CMD_BYTES && frame[next_sync] != (uint8_t)(SPI_CMD_MAGIC & 0xFF)) {
                        next_sync++;
                    }
                    if (next_sync < SPI_CMD_BYTES) {
                        size_t remaining = SPI_CMD_BYTES - next_sync;
                        memmove(frame, &frame[next_sync], remaining);
                        used = remaining;
                        if (used >= 2 && frame[1] != (uint8_t)((SPI_CMD_MAGIC >> 8) & 0xFF)) {
                            if (frame[1] == (uint8_t)(SPI_CMD_MAGIC & 0xFF)) {
                                frame[0] = frame[1];
                                used = 1;
                            } else {
                                used = 0;
                            }
                        }
                    } else {
                        used = 0;
                    }
                }
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

            // Check radar / safety constraints (returns false / clear when ToF is bypassed)
            bool system_safe = !safety_check();

            // Execute movement commands if system is safe and valid unexpired data is present
            if (system_safe && have_command && !latest_cmd.emergency_brake && !command_timeout) {
                float target_speed = (float)latest_cmd.target_motor_effort;
                float target_steering = (float)latest_cmd.target_steering_pulse_us;
                actuator_hal_set_throttle_effort(target_speed);
                actuator_hal_set_steering_pulse_us((uint16_t)target_steering);
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
