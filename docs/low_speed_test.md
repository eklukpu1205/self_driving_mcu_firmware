# Servo and low-speed driving bench test

The normal main.c, platformio.ini, and actuator driver are unchanged.
Build this test from the project root:

    pio run -c platformio-servo-test.ini

Select the test config when uploading too; the normal build is not this test.
The test UF2 is .pio/build/servo_low_speed_test/firmware.uf2.
Do not flash with drive wheels on the floor. After uploading, reset and open
the existing UART serial connection at 115200. This board's SDK defaults to
UART0: GPIO0 TX, GPIO1 RX. Use the existing debug probe's UART bridge if wired,
or a 3.3 V USB-to-UART adapter: GPIO0 -> adapter RX, GPIO1 <- adapter TX,
and common GND. An MCU USB cable alone does not provide this UART console.
Do not connect the adapter's power pin, and do not use 5 V UART logic.
No motor motion is commanded until
f, l, or r is received. Each command starts a two-second burst. Repeated
commands start new bursts. x or space sends zero effort immediately on receipt.
Zero effort is not a verified active brake or a measured physical stop.

Commands:
- c: center, motor stopped
- a/d: side A/B, motor stopped
- f: straight driving
- l/r: driving toward side A/B (confirm which is physically left/right)
- x/space: motor stop and gradual return to center
- h: help (also stops motor)

Settings at the top of src/servo_low_speed_test.c:
- TEST_EFFORT_PERCENT: initially 10 percent open-loop motor effort, not m/s
- DRIVE_BURST_US: initially 2000000 (two seconds)
- SIDE_A_DEG/SIDE_B_DEG: initially 75/105 nominal servo command degrees

Use the existing working motor mode and M1/M3 wiring. The driver sends
negative effort to M1 and positive effort to M3. Verify forward rotation
with the wheels raised; do not assume positive effort means forward on
different wiring. This test does not configure the Hiwonder motor type/mode.

Connections (GPIO labels, not physical header pin numbers):
- GPIO14 -> servo signal (usually white on this chassis)
- GPIO4 -> I2C SDA through the existing suitable I2C level shifter if required
- GPIO5 -> I2C SCL through the existing suitable I2C level shifter if required
- MCU GND -> motor board GND, servo supply negative, and level shifter GND
- Servo positive -> existing correctly regulated, servo-rated supply
- Battery -> existing Hiwonder battery input, preserving working power wiring
- MCU -> existing USB/debug/programming connection for power and serial I/O

Do not connect raw battery or the servo's positive supply to a GPIO or 3V3.
Do not power the servo from MCU 3V3. Do not join two PWM signal outputs.
If the Hiwonder I2C bus is pulled to 5 V, use the 3.3/5 V bidirectional
I2C level shifter: LV=MCU 3V3, HV=Hiwonder 5V, channels SDA and SCL.
The Hiwonder IIC 5V pin is an output, not the raw battery input. Never feed
5 V into MCU GPIOs or the MCU 3V3 rail. This test does not need the ToF or Jetson.

First test with wheels raised. Try c, a, d and check for steering binding.
Try f and verify forward direction and stop. Only then use a clear, level
floor with an accessible physical motor-power disconnect and a helper.
Do not increase effort if the motors stall or hum without turning; stop first
and inspect the setup. Record travel distance and time to measure real speed.

There is no ToF emergency braking, measured speed regulation, hardware motor
watchdog, or guaranteed stop on MCU/serial failure. If I2C fails, firmware
retries zero and locks out driving, but the board might retain its last
command. Disconnect motor power if needed. Never rely on unplugging MCU USB
to stop the car. Firmware has been compile-checked, not tested on hardware.
