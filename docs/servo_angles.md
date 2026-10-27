# Servo angle commands

The new function uses the existing PWM setup and pulse setter:

```c
actuator_hal_set_steering_angle_deg(90.0f);  // Center, 1500 us
actuator_hal_set_steering_angle_deg(75.0f);  // About 1417 us
actuator_hal_set_steering_angle_deg(105.0f); // About 1583 us
```

Call `actuator_hal_init()` once before using the servo. GPIO14, the PWM timing,
motor code, and the existing 1000-2000 us limits are unchanged.

The argument is a servo command in degrees: 0 maps to the minimum pulse,
90 maps to neutral, and 180 maps to the maximum pulse. Values outside that
range are clamped. The function returns false for NaN or infinity and leaves
the previous pulse unchanged. A true result means the command was accepted,
not that the servo position was measured.

These are nominal servo angles, not measured wheel angles. Measure the actual
movement and linkage before using vehicle steering angles. The SPI receiver
currently labels steering values as milliradians. Do not pass them directly
to this function. Conversion to degrees alone also does not calibrate the
servo-to-wheel relationship.

`examples/servo_angle_test.c` is a separate bench test that moves slowly between
75, 90, and 105 degrees. It must replace the active main program for testing,
not compile alongside another main. The angle setter itself does not wait or
add a movement delay. PWM continues running in hardware between commands.

Start with the steering linkage disconnected, as in the earlier bench test.
Power the servo from its rated supply and connect servo and MCU ground.
Keep the motors unpowered for this servo-only test. Do not assume 0 and 180
are safe mechanical endpoints with the linkage installed.

The project's active SPI receiver remains a receive-only demonstration.
Adding this function does not enable actuators or flash new firmware.
