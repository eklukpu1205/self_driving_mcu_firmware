#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { unsigned int index; } i2c_inst_t;
extern i2c_inst_t test_i2c0, test_i2c1;
#define i2c0 (&test_i2c0)
#define i2c1 (&test_i2c1)
unsigned int i2c_init(i2c_inst_t *, unsigned int);
int i2c_write_timeout_us(i2c_inst_t *, uint8_t, const uint8_t *, size_t, bool, unsigned int);
int i2c_read_timeout_us(i2c_inst_t *, uint8_t, uint8_t *, size_t, bool, unsigned int);
