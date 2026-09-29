#pragma once
#include <stdint.h>
#include <stdbool.h>

#define COMMS_CS 13
#define COMMS_SCLK 10
#define COMMS_MOSI 11
#define COMMS_MISO 12


typedef struct {
    float target_speed;
    float target_steering;
    bool emergency_brake;
    uint64_t timestamp_us;
} NvidiaCommand;


typedef struct {
    float actual_speed;
    float actual_steering;
    bool emergency_brake;
    uint64_t timestamp_us;
} McuCommand;

void comms_spi_init(void);
void comms_spi_exchange(NvidiaCommand* cmd_in, const McuCommand* cmd_out);