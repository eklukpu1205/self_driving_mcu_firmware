// Standalone bench example. Use instead of src/main.c, not alongside it.
#include "pico/stdlib.h"
#include "actuator_hal.h"
#include <stdio.h>

static void move_to_angle(int *current_deg, int target_deg) {
    while (*current_deg != target_deg) {
        *current_deg += (*current_deg < target_deg) ? 1 : -1;
        actuator_hal_set_steering_angle_deg((float)*current_deg);
        sleep_ms(20);
    }
}

int main(void) {
    stdio_init_all();
    actuator_hal_init();
    sleep_ms(3000);

    int angle_deg = 90;
    actuator_hal_set_steering_angle_deg(90.0f);

    while (true) {
        printf("Servo command: 75 degrees\n");
        move_to_angle(&angle_deg, 75);
        sleep_ms(1000);

        printf("Servo command: 90 degrees (center)\n");
        move_to_angle(&angle_deg, 90);
        sleep_ms(1000);

        printf("Servo command: 105 degrees\n");
        move_to_angle(&angle_deg, 105);
        sleep_ms(1000);

        move_to_angle(&angle_deg, 90);
        sleep_ms(1000);
    }
}
