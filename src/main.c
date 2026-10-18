#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>

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