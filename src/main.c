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

#define LOOP_PERIOD_US 20000 // 50 Hz



/* STEERING TEST LOOP



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






/* MOTOR TEST LOOP

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




/*
SPI Test Loop


// Parse and print one complete command.
// Returns true only when all four fields are valid.
static bool print_command(const char *line) {
    int sequence;
    int speed;
    int steering;
    int brake;
    char extra;

    // Width limits keep malformed numbers small enough for int.
    // A fifth character means unwanted data followed the four fields.
    const int fields = sscanf(
        line,
        "%5d,%4d,%4d,%1d%c",
        &sequence,
        &speed,
        &steering,
        &brake,
        &extra
    );

    if (fields != 4) {
        return false;
    }

    // Demonstration ranges, not calibrated actuator limits.
    if (sequence < 0 || sequence > 65535 ||
        speed < -500 || speed > 500 ||
        steering < -450 || steering > 450 ||
        (brake != 0 && brake != 1)) {
        return false;
    }

    printf(
        "[RX] seq=%d speed_mm_s=%d steering_mrad=%d brake=%d\n",
        sequence,
        speed,
        steering,
        brake
    );

    // Stage 2 can later pass validated values through safety/control.
    // No motor or servo commands are issued in this demonstration.
    return true;
}

int main(void) {
    // Use the existing working standard-output configuration.
    stdio_init_all();

    // Existing function configures SPI1 slave on GPIO10/11/12/13.
    comms_spi_init();

    spi_set_format(
        spi1,
        8,
        SPI_CPOL_0,
        SPI_CPHA_1,
        SPI_MSB_FIRST
    );

    char line[64];
    unsigned int used = 0;

    bool discard_line = false;
    bool timeout_reported = false;

    uint64_t last_byte_us = time_us_64();
    uint64_t last_valid_us = last_byte_us;

    printf("\nSPI RECEIVER READY - ACTUATORS DISABLED\n");
    printf("SCK=GPIO10, MOSI/RX=GPIO12, CS=GPIO13\n");

    while (true) {
        // Detect receive-FIFO overflow and discard the damaged line.
        if (spi_get_hw(spi1)->ris & SPI_SSPRIS_RORRIS_BITS) {
            spi_get_hw(spi1)->icr = SPI_SSPICR_RORIC_BITS;
            used = 0;
            discard_line = true;
        }

        // Read only available bytes. Never wait for more master clocks.
        if (spi_is_readable(spi1)) {
            const char received = (char)spi_get_hw(spi1)->dr;
            last_byte_us = time_us_64();

            if (received == '\n') {
                line[used] = '\0';

                if (!discard_line && print_command(line)) {
                    last_valid_us = time_us_64();

                    if (timeout_reported) {
                        printf("[COMM] valid reception resumed\n");
                    }

                    timeout_reported = false;
                } else {
                    printf("[RX REJECT] invalid or incomplete command\n");
                }

                used = 0;
                discard_line = false;
            } else if (!discard_line) {
                if (used < sizeof(line) - 1) {
                    line[used++] = received;
                } else {
                    // Ignore an oversized command until its newline.
                    discard_line = true;
                }
            }
        }

        const uint64_t now = time_us_64();

        // Discard a partial command if the remaining bytes never arrive.
        if ((used != 0 || discard_line) &&
            now - last_byte_us > 10000) {
            used = 0;
            discard_line = false;
            printf("[RX REJECT] incomplete command timed out\n");
        }

        // Report once when no valid command has arrived for 250 ms.
        if (!timeout_reported &&
            now - last_valid_us > 250000) {
            printf("[COMM TIMEOUT] no valid command for 250 ms\n");
            timeout_reported = true;
        }

        // Poll continuously so the small SPI FIFO is drained promptly.
        tight_loop_contents();
    }
}



*/