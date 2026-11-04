cmake_minimum_required(VERSION 3.13)

# Use an existing official Pico SDK; no implicit downloads or installation.
if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Set PICO_SDK_PATH to an installed Raspberry Pi Pico SDK")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
project(self_driving_mcu_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(firmware
    src/main.c
    src/actuator_hal.c
    src/comms_spi.c
    src/safety_arbitration.c
    src/sensor_interface.c
    src/hiwonder_driver.c
    src/encoder_feedback.c
)
target_link_libraries(firmware pico_stdlib hardware_spi hardware_pwm hardware_i2c m)



pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 0)


pico_add_extra_outputs(firmware)