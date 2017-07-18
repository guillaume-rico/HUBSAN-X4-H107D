#pragma once

#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "drv_gpio.h"
#include "drv_serial.h"
#include "drv_pwm.h"

void lib_hal_init(void);
// eeprom_read and eeprom_write return number of read/written bytes
size_t eeprom_read_block (void *dst, uint16_t index, size_t size);
size_t eeprom_write_block (const void *src, uint16_t index, size_t size);
void eeprom_commit(void);
