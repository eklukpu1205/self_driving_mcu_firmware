#include "pico/stdlib.h"
#include "hardware/spi.h"
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
#define PARTIAL_TIMEOUT_US 10000u
#define COMMAND_TIMEOUT_US 250000u


/*
// STEERING TEST LOOP



int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    actuator_hal_init();

    sleep_ms(3000);

    sensors_init();
    control_pid_init(); 

    EncoderFeedback WheelFeedback;

    // Data structures for SPI exchange
    NvidiaCommand incoming_cmd = {0};
    McuCommand outgoing_cmd = {0}; 

    EncoderFeedback wheel_feedback; // Raw counts and interval results for bench inspection.
    while (1) {
        // Mark the start time of the loop
        uint64_t loop_start = time_us_64();

        // Check if SPI is readable, then call comms_spi_exchange()

        if (spi_is_readable(spi1)){
            comms_spi_exchange(&incoming_cmd, &outgoing_cmd);
        }    

        

        // Set motor and steering
        if (system_safe && !incoming_cmd.emergency_brake) {
            actuator_set_speed(incoming_cmd.target_speed);
            actuator_set_steering(incoming_cmd.target_steering);
        } else {
            actuator_hal_emergency_halt(); 
        }

        safety_check();

        // Read board-processed encoder totals. First/recovery reads establish a
        // baseline; future consumers must check the return value before using deltas.
        sensors_update(&wheel_feedback);



        // Update outgoing_cmd with the actual speed and steering for the next SPI cycle
        outgoing_cmd.actual_speed = (float)((wheel_feedback.left.speed_mps + wheel_feedback.right.speed_mps) / 2.0);
        outgoing_cmd.actual_steering = incoming_cmd.target_steering;
        outgoing_cmd.emergency_brake = !system_safe;
        outgoing_cmd.timestamp_us = time_us_64();

        // Sleep until loop cycle has passed
        sleep_until(from_us_since_boot(loop_start + LOOP_PERIOD_US));
    }
    return 0;
}

*/




/*

// SPI Test Loop



int main(void) {
    stdio_init_all();
    comms_spi_init();

    uint8_t frame[SPI_CMD_BYTES];
    unsigned used = 0;
    uint32_t valid_total = 0, rejected_total = 0, missing_total = 0;
    uint32_t ticks_total = 0;
    uint32_t valid_this_second = 0, ticks_this_second = 0;
    uint32_t prev_hw_errors = 0, prev_ring_errors = 0;
    uint16_t last_sequence = 0;
    bool have_sequence = false, command_timeout = false;
    SpiCommand latest = {0};
    bool have_command = false;
    uint64_t now = time_us_64();
    uint64_t last_valid_us = now, last_byte_us = now;
    uint64_t next_tick_us = now + SPI_CMD_PERIOD_US;
    uint64_t next_status_us = now + 1000000u;
    // save the first 32 bytes each sec
    uint8_t debug_bytes[32];
    unsigned debug_used = 0;
    uint32_t previous_rx_count = 0;

    puts("SPI BINARY RX READY - ACTUATORS DISABLED");
    puts("SPI1 GP10=SCK GP12=RX GP11=TX GP13=CS; 10 bytes; 50 Hz");

    while (true) {
        uint8_t b;
        while (comms_spi_read_byte(&b)) {
            
            last_byte_us = time_us_64();
            // debug
            if (debug_used < sizeof(debug_bytes)){
                debug_bytes[debug_used++] = b;
            }
            // Stream resync: look for magic bytes 5A A5, then 8 more bytes.
            if (used == 0) {
                if (b != (uint8_t)(SPI_CMD_MAGIC & 0xffu)) continue;
                frame[used++] = b;
                continue;
            }
            if (used == 1) {
                if (b == (uint8_t)(SPI_CMD_MAGIC >> 8)) {
                    frame[used++] = b;
                } else if (b == (uint8_t)(SPI_CMD_MAGIC & 0xffu)) {
                    frame[0] = b; // Another candidate marker begins here.
                } else {
                    used = 0;
                }
                continue;
            }

            frame[used++] = b;
            if (used == SPI_CMD_BYTES) {
                SpiCommand cmd;
                if (spi_command_decode(frame, &cmd)) {
                    latest = cmd;
                    have_command = true;
                    ++valid_total;
                    ++valid_this_second;
                    last_valid_us = time_us_64();
                    command_timeout = false;

                    if (have_sequence) {
                        uint16_t delta = (uint16_t)(cmd.sequence - last_sequence);
                        if (delta > 1u && delta < 32768u) {
                            missing_total += (uint32_t)(delta - 1u);
                        }
                    }
                    last_sequence = cmd.sequence;
                    have_sequence = true;
                    // Print first sample and every new requested command.
                    static int16_t shown_speed = 0, shown_steer = 0;
                    static uint8_t shown_brake = 0;
                    static bool shown_once = false;
                    if (!shown_once || cmd.target_speed_mm_s != shown_speed ||
                        cmd.target_steering_mrad != shown_steer ||
                        cmd.emergency_brake != shown_brake) {
                        printf("[RX] seq=%u speed_mm_s=%d steering_mrad=%d brake=%u\n",
                               (unsigned)cmd.sequence, (int)cmd.target_speed_mm_s,
                               (int)cmd.target_steering_mrad,
                               (unsigned)cmd.emergency_brake);
                        shown_speed = cmd.target_speed_mm_s;
                        shown_steer = cmd.target_steering_mrad;
                        shown_brake = cmd.emergency_brake;
                        shown_once = true;
                    }
                    used = 0;
                } else {
                    ++rejected_total;
                    // Shift by one byte; recover if a marker began inside
                    // a damaged packet, rather than discarding the next one.
                    unsigned keep = 0;
                    for (unsigned i = 1; i < SPI_CMD_BYTES; ++i) {
                        if (frame[i] == (uint8_t)(SPI_CMD_MAGIC & 0xffu)) {
                            if (i + 1u < SPI_CMD_BYTES &&
                                frame[i + 1u] == (uint8_t)(SPI_CMD_MAGIC >> 8)) {
                                keep = SPI_CMD_BYTES - i;
                                for (unsigned j = 0; j < keep; ++j)
                                    frame[j] = frame[i + j];
                                break;
                            }
                            if (i + 1u == SPI_CMD_BYTES) {
                                keep = 1;
                                frame[0] = (uint8_t)(SPI_CMD_MAGIC & 0xffu);
                            }
                        }
                    }
                    used = keep;
                }
            }
        }

        now = time_us_64();
        const uint32_t hw_errors = comms_spi_hw_overrun_count();
        const uint32_t ring_errors = comms_spi_ring_overrun_count();
        if (hw_errors != prev_hw_errors || ring_errors != prev_ring_errors) {
            // Damaged stream: wait for next start marker.
            printf("[SPI OVERRUN] hardware=%lu ring=%lu\n",
                   (unsigned long)hw_errors, (unsigned long)ring_errors);
            used = 0;
            prev_hw_errors = hw_errors;
            prev_ring_errors = ring_errors;
        }
        if (used && now - last_byte_us > PARTIAL_TIMEOUT_US) {
            ++rejected_total;
            used = 0;
            puts("[RX REJECT] partial binary packet timeout");
        }
        if (!command_timeout && now - last_valid_us > COMMAND_TIMEOUT_US) {
            command_timeout = true;
            have_command = false; // Stale commands must NOT be acted upon.
            puts("[COMM TIMEOUT] no valid packet for 250 ms");
        }

        // Distinct 50 Hz MCU control scheduler (TIDR 1-3).
        if (now >= next_tick_us) {
            ++ticks_total;
            ++ticks_this_second;
            // Stage 1: only sample latest validated data. DO NOT actuate.
            (void)latest;
            (void)have_command;
            next_tick_us += SPI_CMD_PERIOD_US;
            if (now >= next_tick_us) next_tick_us = now + SPI_CMD_PERIOD_US;
        }

        // Unconditional heartbeat: monitor remains informative if SPI is silent.
        if (now >= next_status_us) {
            printf("[STATUS] rx_bytes=%lu valid/s=%lu valid_total=%lu "
                   "rejected=%lu missing_seq=%lu ticks/s=%lu hw_ovr=%lu ring_ovr=%lu\n",
                   (unsigned long)comms_spi_rx_byte_count(),
                   (unsigned long)valid_this_second,
                   (unsigned long)valid_total,
                   (unsigned long)rejected_total,
                   (unsigned long)missing_total,
                   (unsigned long)ticks_this_second,
                   (unsigned long)hw_errors,
                   (unsigned long)ring_errors);
                   uint32_t rx_count = comms_spi_rx_byte_count();
                   printf("[RX RATE] bytes/s=%lu\n", (unsigned long) (rx_count - previous_rx_count));
                   previous_rx_count = rx_count;
                   printf("[RAW RX]");
                   for (unsigned i=0; i < debug_used; i++){
                    printf("%02X", (unsigned)debug_bytes[i]);
                   }
                   printf("\n");
                   debug_used = 0;
            valid_this_second = 0;
            ticks_this_second = 0;
            next_status_us += 1000000u;
            if (now >= next_status_us) next_status_us = now + 1000000u;
        }
        tight_loop_contents();
    }
}


*/