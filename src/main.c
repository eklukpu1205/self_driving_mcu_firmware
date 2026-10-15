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

// int main() {
    
//     stdio_init_all();
    
//     // Initialize all hardware peripherals and software states
//     comms_spi_init();
//     actuator_hal_init();

//     bool system_safe = true;
//     sleep_ms(3000);
//     sensors_init();


//     NvidiaCommand incoming_cmd = {0};
//     McuCommand outgoing_cmd = {0}; 
//     EncoderFeedback wheel_feedback; 


//     incoming_cmd.target_speed = 0;
//     incoming_cmd.emergency_brake = false;

   
//     uint16_t current_pulse_us = 1500; 
//     int pulse_step = 25;             

//     // TEST LOOP
//     while (1) {
        
//         if (incoming_cmd.target_speed <= 100){
//             incoming_cmd.target_speed += 2.5;
//         } else {
//             incoming_cmd.target_speed = 2.5;
//         }

//         incoming_cmd.timestamp_us = time_us_64();

//         // Sweep the pulse width directly
//         current_pulse_us += pulse_step;
//         if (current_pulse_us >= 2000) {
//             current_pulse_us = 2000;
//             pulse_step = -25; // Reverse direction
//         } else if (current_pulse_us <= 1000) {
//             current_pulse_us = 1000;
//             pulse_step = 25;  // Reverse direction
//         }

//         uint64_t loop_start = time_us_64();

//         // Check if SPI is readable, then call comms_spi_exchange()
//         if (spi_is_readable(spi1)){
//             comms_spi_exchange(&incoming_cmd, &outgoing_cmd);
//         }    


        
//         sensors_update(&wheel_feedback);
//         system_safe = safety_check();


//         // Set motor and steering using direct pulse width
//         if (system_safe && !incoming_cmd.emergency_brake) {
//             actuator_hal_set_throttle_effort(incoming_cmd.target_speed);
//             actuator_hal_set_steering_pulse_us(current_pulse_us);
//         } else {
//             actuator_hal_emergency_halt(); 
//         }

        

       
        

//         // Update outgoing_cmd with actual values for the next SPI cycle
//         outgoing_cmd.actual_speed = (float)((wheel_feedback.left.speed_mps + wheel_feedback.right.speed_mps) / 2.0);
//         outgoing_cmd.actual_steering = (float)current_pulse_us;
//         outgoing_cmd.emergency_brake = !system_safe;
//         outgoing_cmd.timestamp_us = time_us_64();

//         // Test Status Update
//         printf("\nTEST VEHICLE COMMANDS\n");
//         printf("Current steering pulse %u us\n", current_pulse_us);
//         printf("Current speed command %.3f\n", incoming_cmd.target_speed);

//         // Sleep until loop cycle has passed
//         sleep_until(from_us_since_boot(loop_start + LOOP_PERIOD_US));
//     }
//     return 0;
// }


// eitan servo testing main.
static void move_steering(int *current, int target) {
    while (*current != target) {
        if (*current < target) {
            *current += 5;
            if (*current > target) *current = target;
        } else {
            *current -= 5;
            if (*current < target) *current = target;
        }

        actuator_hal_set_steering_pulse_us((uint16_t)*current);
        sleep_ms(5);
    }
}

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