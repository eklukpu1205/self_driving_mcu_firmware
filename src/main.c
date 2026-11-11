// =====================================================================
//  Drive + Steer + I2C diagnostic test  —  Purdue Proton (RP2350)
//  Scale Autonomous Vehicle Testbed
//
//  Drop in as src/main.c, build with the `proton` env, upload via
//  picoprobe, open serial monitor at 115200. Press 'h' for help.
//
//  SAFETY: wheels raised for first runs, motor power cutoff in reach.
//  Every drive command is a timed burst that ends with a forced stop.
// =====================================================================
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// ---------------- Pins (Proton bench wiring) ----------------
#define MOTOR_I2C   i2c0
#define MOTOR_SDA   4
#define MOTOR_SCL   5
#define TOF_I2C     i2c1
#define TOF_SDA     6
#define TOF_SCL     7
#define SERVO_PIN   14

// ---------------- Hiwonder 4-ch encoder motor driver ----------------
#define HW_ADDR           0x34
#define HW_REG_BATTERY    0x00   // 2 bytes, millivolts, little-endian
#define HW_REG_FIXED_PWM  0x1F   // 4 x int8 effort, -100..100 (M1..M4)
#define HW_REG_ENCODER    0x3C   // 4 x int32 total encoder counts
#define HW_REG_MOTOR_TYPE 0x14   // 0 none, 1 TT, 2 N20, 3 JGB37-520
#define HW_REG_SPEED      0x33   // 4 x int8, closed-loop speed (uses encoders)
#define LEFT_IDX    0            // M1
#define RIGHT_IDX   2            // M3
#define LEFT_SIGN   (-1)         // flip a sign if that wheel spins backwards
#define RIGHT_SIGN  (+1)

// ---------------- VL53L0X ----------------
#define TOF_ADDR    0x29
#define TOF_REG_ID  0xC0         // model ID, expect 0xEE

// ---------------- Limits ----------------
#define EFFORT_DEFAULT   15
#define EFFORT_MAX       40
#define EFFORT_STEP      5
#define BURST_MS         1000
#define REFRESH_MS       100
#define I2C_TIMEOUT_US   10000

#define SERVO_CENTER_US  1500
#define SERVO_MIN_US     1100    // conservative with linkage attached
#define SERVO_MAX_US     1900
#define SERVO_STEP_US    100
#define TURN_OFFSET_US   300

static int      g_effort    = EFFORT_DEFAULT;
static int      g_center_us = SERVO_CENTER_US;
static int      g_servo_us  = SERVO_CENTER_US;
static uint     g_motor_baud = 100000;
static bool     g_closed_loop = false;   // false: 0x1F open-loop PWM, true: 0x33 speed
static uint8_t  g_motor_type = 0;   // 0 = the only type that drove the wheels with PWM 0x1F

// =====================================================================
//  I2C helpers
// =====================================================================
static const char *i2c_err_str(int r) {
    if (r == PICO_ERROR_GENERIC) return "NACK: nobody answered (driver power, GND, SDA/SCL swapped, wrong address)";
    if (r == PICO_ERROR_TIMEOUT) return "TIMEOUT: bus stuck (line held low, no pull-ups, shifter LV/HV unpowered)";
    return "unknown error";
}

static void i2c_setup(i2c_inst_t *i2c, uint sda, uint scl, uint baud) {
    i2c_deinit(i2c);
    uint actual = i2c_init(i2c, baud);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    printf("  I2C%d on SDA=GPIO%u SCL=GPIO%u @ %u Hz\n", i2c_hw_index(i2c), sda, scl, actual);
}

static int reg_write(i2c_inst_t *i2c, uint8_t addr, uint8_t reg,
                     const uint8_t *data, size_t len) {
    uint8_t buf[17];
    if (len > 16) return PICO_ERROR_GENERIC;
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    int r = i2c_write_timeout_us(i2c, addr, buf, len + 1, false, I2C_TIMEOUT_US);
    if (r == (int)(len + 1)) return 0;
    return r < 0 ? r : PICO_ERROR_GENERIC;
}

// Hiwonder expects STOP between register write and read (like Arduino
// Wire endTransmission + requestFrom) -> repeated_start = false.
static int reg_read(i2c_inst_t *i2c, uint8_t addr, uint8_t reg,
                    uint8_t *out, size_t len, bool repeated_start) {
    int r = i2c_write_timeout_us(i2c, addr, &reg, 1, repeated_start, I2C_TIMEOUT_US);
    if (r != 1) return r < 0 ? r : PICO_ERROR_GENERIC;
    r = i2c_read_timeout_us(i2c, addr, out, len, false, I2C_TIMEOUT_US);
    if (r != (int)len) return r < 0 ? r : PICO_ERROR_GENERIC;
    return 0;
}

// Read SDA/SCL as plain inputs. pulls=false reveals external pull-ups.
static void read_lines(uint sda, uint scl, bool pulls, bool *sda_hi, bool *scl_hi) {
    gpio_init(sda);
    gpio_init(scl);
    if (pulls) { gpio_pull_up(sda); gpio_pull_up(scl); }
    else       { gpio_disable_pulls(sda); gpio_disable_pulls(scl); }
    sleep_us(500);
    *sda_hi = gpio_get(sda);
    *scl_hi = gpio_get(scl);
}

static void line_check(i2c_inst_t *i2c, uint sda, uint scl, uint baud, const char *name) {
    bool sda_np, scl_np, sda_p, scl_p;
    read_lines(sda, scl, false, &sda_np, &scl_np);
    read_lines(sda, scl, true,  &sda_p,  &scl_p);
    printf("[%s] idle lines  no-pulls: SDA=%d SCL=%d | internal pull-ups: SDA=%d SCL=%d\n",
           name, sda_np, scl_np, sda_p, scl_p);
    if (sda_np && scl_np)
        printf("  OK: external pull-ups present, bus idle high.\n");
    else if (sda_p && scl_p)
        printf("  WARN: no external pull-ups on Proton side (shifter LV not powered at 3V3?).\n"
               "        Internal ~50k pull-ups only -> unreliable. Check shifter LV/HV/GND.\n");
    else
        printf("  FAIL: %s%s held LOW even with pull-ups -> short to GND, unpowered shifter,\n"
               "        or a stuck device. Try 'r' (bus recovery), then re-check wiring.\n",
               sda_p ? "" : "SDA ", scl_p ? "" : "SCL ");
    i2c_setup(i2c, sda, scl, baud);
}

// Clock out a device stuck mid-byte holding SDA low, then send STOP.
static void bus_recover(i2c_inst_t *i2c, uint sda, uint scl, uint baud, const char *name) {
    gpio_init(sda); gpio_init(scl);
    gpio_pull_up(sda); gpio_pull_up(scl);
    gpio_put(sda, 0); gpio_put(scl, 0);          // output value low; dir controls open-drain
    gpio_set_dir(sda, GPIO_IN);
    for (int i = 0; i < 9; i++) {
        gpio_set_dir(scl, GPIO_OUT); sleep_us(10);  // SCL low
        gpio_set_dir(scl, GPIO_IN);  sleep_us(10);  // SCL released
    }
    // STOP: SDA low while SCL low, release SCL, then release SDA
    gpio_set_dir(scl, GPIO_OUT); sleep_us(10);
    gpio_set_dir(sda, GPIO_OUT); sleep_us(10);
    gpio_set_dir(scl, GPIO_IN);  sleep_us(10);
    gpio_set_dir(sda, GPIO_IN);  sleep_us(10);
    printf("[%s] recovery done: SDA=%d SCL=%d (both should be 1)\n",
           name, gpio_get(sda), gpio_get(scl));
    i2c_setup(i2c, sda, scl, baud);
}

static void i2c_scan(i2c_inst_t *i2c, const char *name) {
    printf("[%s] scanning 0x08-0x77...\n", name);
    int found = 0;
    for (int a = 0x08; a < 0x78; a++) {
        uint8_t d;
        int r = i2c_read_timeout_us(i2c, a, &d, 1, false, 3000);
        if (r >= 0) { printf("  found 0x%02X\n", a); found++; }
        else if (r == PICO_ERROR_TIMEOUT) {
            printf("  TIMEOUT at 0x%02X -> bus stuck, abort scan. Try 'r'.\n", a);
            return;
        }
    }
    printf("  %d device(s) found%s\n", found, found ? "" : " -> check power/GND/SDA-SCL/shifter");
}

// =====================================================================
//  Motor driver
// =====================================================================
static int motor_set(int left, int right) {
    int8_t v[4] = {0, 0, 0, 0};
    v[LEFT_IDX]  = (int8_t)(LEFT_SIGN  * left);
    v[RIGHT_IDX] = (int8_t)(RIGHT_SIGN * right);
    uint8_t reg = g_closed_loop ? HW_REG_SPEED : HW_REG_FIXED_PWM;
    return reg_write(MOTOR_I2C, HW_ADDR, reg, (const uint8_t *)v, 4);
}

// Zero BOTH control registers so switching modes can never leave one running.
static int motor_zero_all(void) {
    int8_t z[4] = {0, 0, 0, 0};
    int r1 = reg_write(MOTOR_I2C, HW_ADDR, HW_REG_FIXED_PWM, (const uint8_t *)z, 4);
    int r2 = reg_write(MOTOR_I2C, HW_ADDR, HW_REG_SPEED,     (const uint8_t *)z, 4);
    return r1 ? r1 : r2;
}

static bool motor_stop_hard(void) {
    for (int i = 0; i < 5; i++) {
        if (motor_zero_all() == 0) return true;
        sleep_ms(20);
    }
    printf("!!! COULD NOT CONFIRM STOP — CUT MOTOR POWER !!!\n");
    return false;
}

static int read_battery_mv(int *mv) {
    uint8_t b[2];
    int r = reg_read(MOTOR_I2C, HW_ADDR, HW_REG_BATTERY, b, 2, false);
    if (r == 0) *mv = b[0] | (b[1] << 8);
    return r;
}

static int read_encoders(int32_t enc[4]) {
    uint8_t b[16];
    int r = reg_read(MOTOR_I2C, HW_ADDR, HW_REG_ENCODER, b, 16, false);
    if (r == 0) memcpy(enc, b, 16);
    return r;
}

// Startup handshake: keep sending stop until the driver ACKs.
static bool motor_connect(int tries) {
    for (int i = 1; i <= tries; i++) {
        int r = motor_set(0, 0);
        if (r == 0) {
            printf("Motor driver 0x34 ACKed zero-effort (try %d).\n", i);
            int rt = reg_write(MOTOR_I2C, HW_ADDR, HW_REG_MOTOR_TYPE, &g_motor_type, 1);
            printf("Motor type -> %d at startup: %s\n", g_motor_type, rt == 0 ? "OK" : i2c_err_str(rt));
            return true;
        }
        printf("  try %d/%d: %d %s\n", i, tries, r, i2c_err_str(r));
        sleep_ms(250);
    }
    return false;
}

static void motor_health(void) {
    printf("---- Motor I2C health ----\n");
    line_check(MOTOR_I2C, MOTOR_SDA, MOTOR_SCL, g_motor_baud, "motor");
    int mv, r;
    r = read_battery_mv(&mv);
    if (r == 0) printf("Battery (driver ADC): %d mV%s\n", mv,
                       mv < 9000 ? "  <- LOW or driver battery not on" : "");
    else        printf("Battery read FAILED: %d %s\n", r, i2c_err_str(r));
    r = motor_set(0, 0);
    printf("Zero-effort write: %s\n", r == 0 ? "OK" : i2c_err_str(r));
    int32_t e[4];
    r = read_encoders(e);
    if (r == 0) printf("Encoders M1..M4: %ld %ld %ld %ld\n",
                       (long)e[0], (long)e[1], (long)e[2], (long)e[3]);
    else        printf("Encoder read FAILED: %d %s\n", r, i2c_err_str(r));
}

// 200 reads in a row: catches loose wires / intermittent -1.
static void motor_stress(void) {
    int ok = 0, nack = 0, tmo = 0, mv;
    printf("Stress: 200 battery reads (wiggle the wires while this runs)...\n");
    for (int i = 0; i < 200; i++) {
        int r = read_battery_mv(&mv);
        if (r == 0) ok++;
        else if (r == PICO_ERROR_TIMEOUT) tmo++;
        else nack++;
        sleep_ms(10);
    }
    printf("  OK=%d  NACK=%d  TIMEOUT=%d  -> %s\n", ok, nack, tmo,
           ok == 200 ? "bus solid" :
           ok == 0   ? "bus dead, run 'i'" : "INTERMITTENT: loose wire/ground or bad breadboard row");
}

// =====================================================================
//  Servo
// =====================================================================
static void servo_write(int us) {
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    g_servo_us = us;
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)us);   // 1 tick = 1 us
    printf("Servo: %d us (center %d)\n", us, g_center_us);
}

static void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f); // 1 MHz
    pwm_config_set_wrap(&cfg, 19999);                                       // 20 ms = 50 Hz
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(SERVO_PIN, SERVO_CENTER_US);
    pwm_set_enabled(slice, true);
    g_servo_us = SERVO_CENTER_US;
}

// =====================================================================
//  ToF quick check (presence + ID only)
// =====================================================================
static void tof_check(void) {
    uint8_t id = 0;
    int r = reg_read(TOF_I2C, TOF_ADDR, TOF_REG_ID, &id, 1, true);
    if (r == 0) printf("VL53L0X ID = 0x%02X %s\n", id, id == 0xEE ? "(OK)" : "(unexpected!)");
    else        printf("VL53L0X read FAILED: %d %s\n", r, i2c_err_str(r));
}

// =====================================================================
//  Drive burst
// =====================================================================
static void drive_burst(int left, int right, int servo_us, bool recenter, const char *label) {
    int r = motor_set(0, 0);
    if (r) { printf("Drive refused, driver not answering: %s. Run 'i'.\n", i2c_err_str(r)); return; }

    servo_write(servo_us);
    sleep_ms(150);                       // let steering settle first

    int32_t e0[4], e1[4];
    bool enc_ok = (read_encoders(e0) == 0);

    printf(">> %s  L=%d R=%d for %d ms  mode=%s  (x/space = stop)\n", label, left, right,
           BURST_MS, g_closed_loop ? "SPEED 0x33" : "PWM 0x1F");
    printf("   t(ms)  batt(mV)   M1     M3\n");
    absolute_time_t start = get_absolute_time();
    absolute_time_t end = make_timeout_time_ms(BURST_MS);
    int sends = 0;
    while (!time_reached(end)) {
        r = motor_set(left, right);
        if (r) { printf("I2C error mid-burst after %d sends: %d %s\n", sends, r, i2c_err_str(r)); break; }
        sends++;
        int mv = -1; int32_t e[4] = {0};
        read_battery_mv(&mv);
        bool eok = (read_encoders(e) == 0);
        // newlib-nano printf has no %lld -> cast everything to int
        printf("   %5d  %6d  %6d %6d\n",
               (int)(absolute_time_diff_us(start, get_absolute_time()) / 1000), mv,
               eok ? (int)(e[LEFT_IDX]  - e0[LEFT_IDX])  : 0,
               eok ? (int)(e[RIGHT_IDX] - e0[RIGHT_IDX]) : 0);
        int c = getchar_timeout_us(0);
        if (c == 'x' || c == ' ') { printf("Stopped by user.\n"); break; }
        sleep_ms(REFRESH_MS);
    }
    motor_stop_hard();
    printf("<< stop sent (%d commands OK)\n", sends);

    if (enc_ok && read_encoders(e1) == 0)
        printf("Encoder delta  M1=%ld  M3=%ld  (0 = wheel did not turn or no encoder)\n",
               (long)(e1[LEFT_IDX] - e0[LEFT_IDX]), (long)(e1[RIGHT_IDX] - e0[RIGHT_IDX]));
    if (recenter) servo_write(g_center_us);
}

// =====================================================================
//  UI
// =====================================================================
static void help(void) {
    printf("\n=========== DRIVE / STEER / I2C TEST ===========\n"
           " I2C:   s scan both buses   i motor health   p stress test\n"
           "        r bus recovery      f toggle motor I2C 100k/50k   t ToF ID\n"
           " Motor: m cycle motor type (now %d)   v toggle PWM 0x1F / SPEED 0x33 (now %s)\n"
           " Drive: w forward  b backward  q fwd+left  e fwd+right  (%d ms bursts)\n"
           "        + / - effort (now %d%%, max %d%%)   x or SPACE stop\n"
           " Steer: a left  d right  c center  C set current as center\n"
           " Wheels RAISED for first runs. Keep motor power cutoff in reach.\n"
           "================================================\n",
           g_motor_type, g_closed_loop ? "SPEED" : "PWM", BURST_MS, g_effort, EFFORT_MAX);
}

int main(void) {
    stdio_init_all();
    servo_init();                        // servo centered immediately
    sleep_ms(2000);                      // time to open the serial monitor
    printf("\nBoot. Setting up I2C...\n");

    i2c_setup(MOTOR_I2C, MOTOR_SDA, MOTOR_SCL, g_motor_baud);
    i2c_setup(TOF_I2C, TOF_SDA, TOF_SCL, 100000);

    line_check(MOTOR_I2C, MOTOR_SDA, MOTOR_SCL, g_motor_baud, "motor");
    i2c_scan(MOTOR_I2C, "motor i2c0");
    i2c_scan(TOF_I2C,   "tof i2c1");

    if (!motor_connect(20))
        printf("Motor driver NOT responding. Turn on chassis battery, then press 'i'.\n");
    help();

    while (true) {
        int c = getchar_timeout_us(100000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        switch (c) {
            case 'h': help(); break;
            case 's': i2c_scan(MOTOR_I2C, "motor i2c0"); i2c_scan(TOF_I2C, "tof i2c1"); break;
            case 'i': motor_health(); break;
            case 'p': motor_stress(); break;
            case 't': tof_check(); break;
            case 'r':
                bus_recover(MOTOR_I2C, MOTOR_SDA, MOTOR_SCL, g_motor_baud, "motor");
                bus_recover(TOF_I2C, TOF_SDA, TOF_SCL, 100000, "tof");
                break;
            case 'f':
                g_motor_baud = (g_motor_baud == 100000) ? 50000 : 100000;
                i2c_setup(MOTOR_I2C, MOTOR_SDA, MOTOR_SCL, g_motor_baud);
                break;
            case '+': case '=':
                g_effort += EFFORT_STEP; if (g_effort > EFFORT_MAX) g_effort = EFFORT_MAX;
                printf("Effort %d%%\n", g_effort); break;
            case '-':
                g_effort -= EFFORT_STEP; if (g_effort < EFFORT_STEP) g_effort = EFFORT_STEP;
                printf("Effort %d%%\n", g_effort); break;
            case 'w': drive_burst( g_effort,  g_effort, g_servo_us, false, "FORWARD");  break;
            case 'b': drive_burst(-g_effort, -g_effort, g_servo_us, false, "BACKWARD"); break;
            case 'q': drive_burst( g_effort,  g_effort, g_center_us - TURN_OFFSET_US, true, "FWD + LEFT");  break;
            case 'e': drive_burst( g_effort,  g_effort, g_center_us + TURN_OFFSET_US, true, "FWD + RIGHT"); break;
            case 'a': servo_write(g_servo_us - SERVO_STEP_US); break;
            case 'd': servo_write(g_servo_us + SERVO_STEP_US); break;
            case 'c': servo_write(g_center_us); break;
            case 'C': g_center_us = g_servo_us; printf("Center set to %d us\n", g_center_us); break;
            case 'm': {
                g_motor_type = (uint8_t)((g_motor_type + 1) % 4);
                motor_stop_hard();
                int r = reg_write(MOTOR_I2C, HW_ADDR, HW_REG_MOTOR_TYPE, &g_motor_type, 1);
                printf("Motor type -> %d (0 none,1 TT,2 N20,3 JGB37): %s\n",
                       g_motor_type, r == 0 ? "OK" : i2c_err_str(r));
            } break;
            case 'v':
                motor_stop_hard();
                g_closed_loop = !g_closed_loop;
                printf("Drive mode -> %s\n", g_closed_loop ? "SPEED 0x33 (closed loop)" : "PWM 0x1F (open loop)");
                break;
            case 'x': case ' ': motor_stop_hard(); printf("STOP\n"); break;
            default: break;
        }
    }
}