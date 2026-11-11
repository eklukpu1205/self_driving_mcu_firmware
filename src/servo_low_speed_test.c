// Build with platformio-servo-test.ini. The normal firmware does not use this main.
#ifdef SERVO_LOW_SPEED_TEST
#include "pico/stdlib.h"
#include "actuator_hal.h"
#include <stdio.h>

// These are bench-test choices, not calibrated speed or wheel angles.
#define TEST_EFFORT_PERCENT 10.0f
#define DRIVE_BURST_US 2000000ULL
#define LOOP_MS 20
#define CENTER_DEG 90
#define SIDE_A_DEG 75
#define SIDE_B_DEG 105

static bool fault;
static bool driving;
static uint64_t drive_deadline;
static int current_angle = CENTER_DEG;
static int target_angle = CENTER_DEG;

static bool stop_motor(void) {
    driving = false;
    if (!actuator_hal_set_throttle_effort(0.0f)) {
        fault = true;
        return false;
    }
    return true;
}

static void print_help(void) {
    puts("\nServo + motor bench test. No obstacle detection or speed feedback.");
    puts("c=center, a=steer side A, d=steer side B (motors stopped)");
    puts("f=drive straight, l=drive side A, r=drive side B (2 seconds)");
    puts("x or SPACE=send motor stop; h=help. Commands need no Enter.");
    puts("Keep a physical motor-power disconnect within reach.");
}

static void handle_command(int ch) {
    if (ch == '\r' || ch == '\n') return;
    // Stop before printing or accepting a new movement command.
    if (!stop_motor()) return;
    if (fault) return;
    switch (ch) {
        case 'a': target_angle = SIDE_A_DEG; break;
        case 'd': target_angle = SIDE_B_DEG; break;
        case 'c': case 'x': case ' ': target_angle = CENTER_DEG; break;
        case 'f': target_angle = CENTER_DEG; driving = true; break;
        case 'l': target_angle = SIDE_A_DEG; driving = true; break;
        case 'r': target_angle = SIDE_B_DEG; driving = true; break;
        case 'h': print_help(); break;
        default: puts("Unknown command. Motor stopped. h=help."); break;
    }
    drive_deadline = time_us_64() + DRIVE_BURST_US;
}

int main(void) {
    stdio_init_all();
    actuator_hal_init();
    stop_motor();
    actuator_hal_set_steering_angle_deg(CENTER_DEG);
    print_help();

    while (true) {
        if (fault) {
            // A failed I2C stop does NOT prove the motor has stopped.
            // Retry zero effort, but latch out all driving until MCU reset.
            actuator_hal_set_throttle_effort(0.0f);
            static bool reported;
            if (!reported) {
                puts("I2C FAILURE: disconnect motor power. Driving locked until reset.");
                reported = true;
            }
            sleep_ms(100);
            continue;
        }

        if (driving && time_us_64() >= drive_deadline) {
            if (!stop_motor()) continue;
            target_angle = CENTER_DEG;
            puts("Burst complete. Zero motor effort sent.");
        }

        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) handle_command(ch);
        if (fault) continue;

        // One degree per loop keeps the same gradual steering approach.
        if (current_angle < target_angle) current_angle++;
        else if (current_angle > target_angle) current_angle--;
        actuator_hal_set_steering_angle_deg((float)current_angle);

        // Refresh the motor command; never call the normal SPI/ToF routines here.
        if (!actuator_hal_set_throttle_effort(driving ? TEST_EFFORT_PERCENT : 0.0f)) {
            fault = true;
            stop_motor();
        }
        sleep_ms(LOOP_MS);
    }
}
#endif
