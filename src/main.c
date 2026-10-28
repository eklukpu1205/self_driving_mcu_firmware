#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>


// Include subsystem headers
#include "comms_spi.h"
#include "safety_arbitration.h"
#include "sensor_interface.h"
#include "actuator_hal.h"
#include "encoder_feedback.h"
#include "hiwonder_driver.h"

#define LOOP_PERIOD_US 20000 // 50 Hz
#define MAX_LINE 64u
#define PARTIAL_TIMEOUT_US 20000u
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

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool print_command(char *line) {
    if (line[0] != '@') return false;
    char *star = strchr(line, '*');
    if (!star || strlen(star) != 3u) return false; // *XX\0

    int high = hex_value(star[1]);
    int low  = hex_value(star[2]);
    if (high < 0 || low < 0) return false;

    uint8_t checksum = 0;
    for (char *p = line + 1; p < star; ++p) checksum ^= (uint8_t)*p;
    if (checksum != (uint8_t)((high << 4) | low)) return false;

    *star = '\0';
    unsigned int sequence = 0, brake = 0;
    int speed = 0, steering = 0, consumed = -1;
    int fields = sscanf(line + 1, "%5u,%4d,%4d,%1u%n",
                        &sequence, &speed, &steering, &brake, &consumed);
    if (fields != 4 || consumed < 0 || line[1 + consumed] != '\0') return false;
    if (sequence > 65535u || speed < -500 || speed > 500 ||
        steering < -450 || steering > 450 || brake > 1u) return false;

    printf("[RX] seq=%u speed_mm_s=%d steering_mrad=%d brake=%u\n",
           sequence, speed, steering, brake);
    return true;
}

int main(void) {
    stdio_init_all();
    comms_spi_init();
    printf("\nSPI TEXT RECEIVER READY - ACTUATORS DISABLED\n");
    printf("SPI1: SCK=10 MOSI/RX=12 MISO/TX=11 CS=13\n");

    char line[MAX_LINE];
    unsigned int used = 0;
    bool receiving = false;
    bool timeout_reported = false;
    uint64_t last_byte_us = time_us_64();
    uint64_t last_valid_us = last_byte_us;
    uint32_t last_error_count = 0;
    uint8_t byte;

    while (true) {
        // Read buffered bytes; the SPI interrupt continues receiving during printf.
        while (comms_spi_read_byte(&byte)) {
            last_byte_us = time_us_64();
            if (byte == 0) continue; // Fixed-slot zero padding.
            if (byte == '@') {
                used = 0;
                receiving = true; // Start of a new text frame.
                line[used++] = '@';
                continue;
            }
            if (!receiving) continue;
            if (byte == '\n') {
                line[used] = '\0';
                if (print_command(line)) {
                    last_valid_us = time_us_64();
                    if (timeout_reported) puts("[COMM] valid reception resumed");
                    timeout_reported = false;
                } else {
                    puts("[RX REJECT] checksum, format, or range error");
                }
                used = 0;
                receiving = false;
            } else if (byte >= 32u && byte <= 126u && used < MAX_LINE - 1u) {
                line[used++] = (char)byte;
            } else {
                // Bad character or long frame: wait for the next '@' to resync.
                used = 0;
                receiving = false;
            }
        }

        uint32_t errors = comms_spi_error_count();
        if (errors != last_error_count) {
            printf("[SPI ERROR] hardware/ring overflow: count=%lu\n",
                   (unsigned long)errors);
            last_error_count = errors;
            used = 0;
            receiving = false;
        }

        uint64_t now_us = time_us_64();
        if (receiving && now_us - last_byte_us > PARTIAL_TIMEOUT_US) {
            puts("[RX REJECT] incomplete frame timeout");
            used = 0;
            receiving = false;
        }
        if (!timeout_reported && now_us - last_valid_us > COMMAND_TIMEOUT_US) {
            puts("[COMM TIMEOUT] no valid command for 250 ms");
            timeout_reported = true;
        }
        tight_loop_contents();
    }
}