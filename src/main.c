#include "pico/stdlib.h"
#include "hardware/spi.h"

// Include subsystem headers
#include "comms_spi.h"
#include "safety_arbitration.h"
#include "sensor_interface.h"
#include "actuator_hal.h"
#include "encoder_feedback.h"
#include "hiwonder_driver.h"


#define LOOP_PERIOD_US 20000 // 50 Hz

// Safety check and pass Jetson's target speed and radar distance through safety_arbitrate_speed()

system_safe = true;



int main() {
    // Initialize USB/UART for standard printf debugging
    stdio_init_all();
    
    // Initialize all hardware peripherals and software states
    comms_spi_init();
    actuator_hal_init();

    sleep_ms(3000);

    sensors_init();
    control_pid_init(); 

    EncoderFeedback WheelFeedback;

    // Data structures for SPI exchange
    NvidiaCommand incoming_cmd = {0};
    McuCommand outgoing_cmd = {0}; 

    EncoderFeedback wheel_feedback; // Raw counts and interval results for bench inspection.



    // TEST LOOP

    while (1) {
        
        incoming_cmd.target_speed = 0;
        incoming_cmd.target_steering = PULSE_NEUTRAL_US;
        incoming_cmd.emergency_brake = false;
        incoming_cmd.timestamp_us = time_us_64();
        

        uint64_t loop_start = time_us_64();

        // Check if SPI is readable, then call comms_spi_exchange()

        if (spi_is_readable(spi1)){
            comms_spi_exchange(&incoming_cmd, &outgoing_cmd);
        }    

        

        // Set motor and steering
        if (system_safe && !incoming_cmd.emergency_brake) {
            actuator_hal_set_throttle_effort(incoming_cmd.target_speed);
            actuator_hal_set_steering_rad(incoming_cmd.target_steering);
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