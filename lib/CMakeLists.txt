cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(self_driving_mcu_firmware C)
pico_sdk_init()

add_executable(firmware
    main.c
    comms_spi.c
    safety_arbitration.c
    actuator_hal.c
    sensor_interface.c
    control_loops.c
)

target_link_libraries(firmware pico_stdlib hardware_spi hardware_pwm)
pico_add_extra_outputs(firmware)