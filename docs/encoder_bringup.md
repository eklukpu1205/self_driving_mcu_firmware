# Minimal encoder feedback

## Data path

Rear-wheel A/B signals -> Hiwonder four-channel board -> I2C total counts ->
RP2040 previous/current subtraction -> optional per-interval distance and speed.
The MCU does not decode A/B edges. Motor control and servo code are unchanged.

## Count format and evidence

Hiwonder's [Arduino tutorial](https://docs.hiwonder.com/projects/Ackermann-Chassis/en/latest/docs/3_Arduino_Version_formatted.html)
identifies I2C address 0x34 and total-count register 60.
[PX4's Hiwonder driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/hiwonder_emm/HiwonderEMM.cpp)
uses int32_t counts and assembles four little-endian bytes per channel from a
16-byte read. Our driver follows that signed format in M1..M4 order, including
negative values. PX4 is implementation evidence, not vendor verification of the
installed revision. Confirm negative counts and channel order on the bench.

The subtraction casts current to int64_t before subtracting previous int32_t.
This prevents C signed arithmetic overflow; it does not reconstruct hardware
counter wraparound. Reset/overflow semantics are unverified. Reset estimator
history after a known board reset; restart a bench run before counter limits.
There is no automatic reset detection or overflow compensation.

## Configuration

`src/encoder_config.h` contains the user-confirmed I2C0 GPIO4 SDA/GPIO5 SCL,
provisional left=M1/right=M3, and per-wheel calibration. Address is 0x34 in
`hiwonder_driver.h`. GPIO14 is reserved by the user for servo PWM, outside this
feature. The 100 kHz bus and 3 ms per-transfer timeout are host choices.

Start with forward_sign=+1 (no correction). Change a wheel to -1 only when its
observed forward rotation decreases counts. Raw counts and raw deltas remain
unchanged for bench inspection; sign correction applies to distance/speed only.
Leave counts_per_wheel_revolution and circumference zero until measured.
Measure wheel revolutions, not motor-shaft revolutions; do not apply gearing or
quadrature multipliers again. Calibration is independent for each wheel.

## Small interface

EncoderState holds two previous counts, a previous microsecond timestamp, and
one boolean indicating whether a baseline exists. EncoderFeedback is an output
value owned by the caller: left/right raw_count, delta_count, distance_change_m,
speed_mps, and dt_s. There is no global snapshot, expiry policy, lifetime distance
or collection of status flags. Pointer arguments must be non-null.

`sensors_update(&wheel_feedback)` reads all four totals and selects the two
configured channels. It runs in the existing nominal 20 ms main loop. Timestamp
is host reception time, not a board-latched measurement timestamp.

- First successful read: save counts/time, expose raw counts, return false.
- Later successful read: delta=current-previous, dt=(now-previous)/1e6; return true.
- Physical conversion: distance_change_m=sign*delta/counts_per_wheel_revolution*
  circumference_m; speed_mps=distance_change_m/dt_s.
- Missing/invalid scale: still return raw deltas, but distance/speed are NAN.
- Failed I2C read: clear output, set metric values NAN, discard baseline, return false.
- Non-increasing timestamp: expose current raw counts, discard baseline, return false.
- After either error: the next good read is baseline-only, not a speed update.

True means deltas/dt are available, not that calibration has been measured.
False means no delta; output must not be treated as a new motion measurement.
No configurable freshness expiry is applied. The existing speed accessor returns
only the most recent update's mean speed (or NAN), so call update before using it.

## Bench sequence

1. Verify common ground, 3.3 V-compatible pull-ups and GPIO4/5 wiring.
2. With propulsion disabled, inspect main's wheel_feedback immediately after
   sensors_update. Rotate each wheel independently to verify M1/M3 mapping.
3. Verify negative/positive totals and forward direction; adjust signs if needed.
4. Measure counts over several wheel revolutions and wheel rollout circumference.
5. Enter calibration, then compare interval distance/speed against measurements.
6. Disconnect/reconnect I2C and verify the first good read only re-baselines.

No motor/configuration registers are written. Only the encoder register selector
is sent. Whether motor-type initialization is needed for counts, board reset
behavior, count coherence across channels, and attainable sampling rate still
need verification. Coordinate any motor-type setup with the motor developer.
No console logger or hardware test is included.

## Checks and build

Host tests use synthetic calibration and fake I2C/GPIO/time. They verify signed
byte decoding, partial/failed reads, baseline handling, raw subtraction, measured
dt, sign correction, missing calibration, bad timing, and recovery.

```text
gcc -std=c11 -Wall -Wextra -Werror -Itest/host -Isrc test/host/test_encoder.c src/encoder_feedback.c src/hiwonder_driver.c src/sensor_interface.c -lm -o test/host/encoder_tests.exe
./test/host/encoder_tests.exe
```

The existing PlatformIO config selects picosdk, which the locally installed
raspberrypi platform 1.20.0 rejects. Project-root CMakeLists.txt includes the two
encoder source modules and hardware_i2c; it needs an existing official Pico SDK
and ARM toolchain. No dependencies are downloaded automatically.

```text
cmake -S . -B build -DPICO_SDK_PATH=<sdk-path> -DPICO_BOARD=pico
cmake --build build
```

The old lib/CMakeLists.txt is not used by this root entry. Host tests do not
establish a successful firmware link or physical board operation.

## Integration in the Desktop repository

The existing actuator module and its main-loop initialization sequence are
preserved. It also uses I2C0 at 100 kHz; encoder setup configures GPIO4/5 on that
same peripheral and sends no motor commands. Keep actuator/encoder transactions
serialized in the main loop. GPIO14 steering remains owned by actuator_hal.

Full firmware compilation also requires repair of pre-existing actuator_hal.c
syntax errors, missing constants and the missing actuator_hal_init definition.
Those teammate-owned issues are not repaired by this encoder transfer.
