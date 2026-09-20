#pragma once
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    float target_speed;
    float target_steering;
} NvidiaCommand;


typedef struct {
    float actual_speed;
    float actual_steering;
} McuCommand;

void comms_spi_init(void);
void comms_spi_exchange(NvidiaCommand* cmd_in, const McuCommand* cmd_out);