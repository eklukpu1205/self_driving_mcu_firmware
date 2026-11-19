#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
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

// Query the motor type from the Hiwonder motor board on i2c0 (addr 0x34, reg 0x14)
static int query_motor_type(uint8_t *type_out) {
    uint8_t reg = MOTOR_TYPE_ADDR;
    if (i2c_write_timeout_us(i2c0, HI_WONDER_I2C_ADDR, &reg, 1, true, 5000) != 1) {
        return -1;
    }
    uint8_t val = 0xFF;
    if (i2c_read_timeout_us(i2c0, HI_WONDER_I2C_ADDR, &val, 1, false, 5000) != 1) {
        return -2;
    }
    if (type_out) {
        *type_out = val;
    }
    return 0;
}

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

    // Query and verify motor type at startup from i2c0 (HI_WONDER_I2C_ADDR)
    uint8_t startup_motor_type = 0xFF;
    int err = query_motor_type(&startup_motor_type);
    if (err == 0) {
        printf("[Main]: Motor type initialized on i2c0 (addr 0x%02X, reg 0x%02X): %u\n",
               HI_WONDER_I2C_ADDR, MOTOR_TYPE_ADDR, startup_motor_type);
    } else {
        printf("[Main]: Failed to read initial motor type from i2c0 (err=%d)\n", err);
    }

    // Initialize sensors (encoders and ToF)
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

            // Check radar / safety constraints (evaluates ToF distance internally)
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

            // Query motor type periodically during execution (every 50 ticks = 1 Hz)
            static uint32_t motor_type_query_count = 0;
            if (++motor_type_query_count % 50 == 0) {
                uint8_t exec_motor_type = 0xFF;
                if (query_motor_type(&exec_motor_type) == 0) {
                    printf("[Main]: Motor type during execution: %u\n", exec_motor_type);
                } else {
                    printf("[Main]: Failed to read motor type during execution\n");
                }
            }
        }

        // Put CPU to sleep until the next interrupt 
        tight_loop_contents();
    }
    return 0;
}
