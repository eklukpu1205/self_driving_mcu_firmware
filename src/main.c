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

    int current = 1500;
    actuator_hal_set_steering_pulse_us(current);
    sleep_ms(1000);

    while (true) {
        printf("Move to 700\n");
        move_steering(&current, 700);
        sleep_ms(1000);

        printf("Move to center\n");
        move_steering(&current, 1500);
        sleep_ms(1000);

        printf("Move to 2300\n");
        move_steering(&current, 2300);
        sleep_ms(1000);

        printf("Move to center\n");
        move_steering(&current, 1500);
        sleep_ms(1000);
    }
}

*/






/* 
// MOTOR TEST LOOP

// Define test timing parameters
#define TEST_DELAY_MS 2000

void test_motor_constant_velocity() {
    printf("\nMotor Test\n");
    
    float test_speed = 50.0f; 
    
    printf("Setting motor to %.1f%% effort\n", test_speed);
    int test_duration_ms = TEST_DELAY_MS * 2; 
    int iterations = test_duration_ms / 20; 
    
    for (int i = 0; i < iterations; i++) {
        actuator_hal_set_throttle_effort(test_speed);
        sleep_ms(20);
    }
    actuator_hal_set_throttle_effort(0.0f);
    
    // Hold constant velocity
    sleep_ms(TEST_DELAY_MS * 2);
}

void test_motor_emergency_halt() {
    printf("\nEmergency Stop Test\n");

    
    for (int i = 0; i < 250; i++) {
        actuator_hal_set_throttle_effort(50.0f);
        sleep_ms(20);
    }
    
    uint64_t stop_start_time = time_us_64();
    
    // Trigger emergency stop 
    actuator_hal_emergency_halt();
    
    uint64_t stop_end_time = time_us_64();
    uint64_t elapsed_time = stop_end_time - stop_start_time;
    
    printf("Emergency halt executed. Execution time: %.10f us\n", elapsed_time);
    
    sleep_ms(TEST_DELAY_MS);
}

void test_motor_ramp() {
    printf("\nMotor Ramp Test\n");

    // Gradually ramp up motor speed
    for (float speed = 0.0f; speed <= 100.0f; speed += 10.0f) {
        printf("Ramping up: %.1f%%\n", speed);
       
        for (int i = 0; i < 25; i++) {
            actuator_hal_set_throttle_effort(speed);
            sleep_ms(20);
        }
    }
    
    // Gradually ramp down
    for (float speed = 100.0f; speed >= 0.0f; speed -= 10.0f) {
        printf("Ramping down: %.1f%%\n", speed);
        
        for (int i = 0; i < 25; i++) {
            actuator_hal_set_throttle_effort(speed);
            sleep_ms(20);
        }
    }
    actuator_hal_set_throttle_effort(0.0f);
}

int main(void) {
    stdio_init_all();
    
    sleep_ms(3000); 
    
    printf("Starting Motor Drive Test\n");
    
    actuator_hal_init();

    while (true) {
        test_motor_constant_velocity();
        test_motor_emergency_halt();
        test_motor_ramp();
        
        printf("\nTEST SEQUENCE COMPLETE\n");
        sleep_ms(5000);
    }
    
    return 0;
}

*/



/*

// EMERGENCY BRAKING TEST LOOP

// Define test timing parameters
#define TEST_DELAY_MS 2000



static bool emergency_stop_latched = false;
void test_motor_constant_velocity(void) {
    const float test_speed = 50.0f;
    const int iterations = 2000;

    for (int i = 0; i < iterations; i++) {
        if (emergency_stop_latched || safety_check()) {
            actuator_hal_emergency_halt();
            emergency_stop_latched = true;

            printf("Emergency braking triggered by ToF\n");
            return;
        }

        actuator_hal_set_throttle_effort(test_speed);
        sleep_ms(20);
    }

    actuator_hal_set_throttle_effort(0.0f);
}



int main(void) {
    stdio_init_all();
    
    sleep_ms(3000); 
    
    printf("\nEmergency Brake Test\n");
    sensors_init();
    
    actuator_hal_init();

   while (true) {
    test_motor_constant_velocity();

    if (emergency_stop_latched) {
        printf("Motor remains stopped. Reset the MCU.\n");

        while (true) {
            sleep_ms(1000);
        }
    }

    sleep_ms(1000);
}
    
    return 0;
}

*/





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
<<<<<<< HEAD
                    printf("%02X", (unsigned)debug_bytes[i]);
=======
                        printf(" %02X", (unsigned)debug_bytes[i]);
>>>>>>> 7e036739ae2a93b9e031be4a62a002535a79b0c4
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