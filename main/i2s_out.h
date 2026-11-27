/*
  i2s_out.h

  Part of grblHAL driver for ESP32

  Copyright (c) 2024-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

  Inspired by work:
    Copyright (c) 2020 - Michiyasu Odaki, for Grbl_ESP32.
    Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm, for Marlin 3D Printer Firmware.
*/

#pragma once

#include "driver.h"

#ifdef USE_I2S_OUT

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define I2SO(n) (I2S_OUT_PIN_BASE + n)

#if defined(I2S_OUT_NUM_BITS)
#if (I2S_OUT_NUM_BITS != 16) && (I2S_OUT_NUM_BITS != 32)
#error "I2S_OUT_NUM_BITS should be 16 or 32"
#endif
#else
#define I2S_OUT_NUM_BITS 32
#endif

#ifndef I2S_OUT_USEC_PER_PULSE
#define I2S_OUT_USEC_PER_PULSE 2
#endif

typedef union {
    uint8_t val;
    struct {
        uint8_t dir_changed :1,
                step        :1,
                unused      :6;
    };
} i2s_pulse_t;

static inline void i2s_gpio_cfg (uint8_t gpio, uint32_t signal_idx)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction((gpio_num_t)gpio, GPIO_MODE_OUTPUT);
    gpio_matrix_out(gpio, signal_idx, 0, 0);
    gpio_pad_set_drv(gpio, 3);
}

static inline uint32_t i2s_sample_cfg (float us)
{
    uint32_t samples = (uint32_t)ceilf(us);

    if(samples % I2S_OUT_USEC_PER_PULSE)
        samples += I2S_OUT_USEC_PER_PULSE - samples % I2S_OUT_USEC_PER_PULSE;

    return samples / I2S_OUT_USEC_PER_PULSE;
}

uint32_t i2s_out_init (settings_t *settings);
void i2s_out_write(uint8_t pin, bool val);
bool i2s_out_state (uint8_t pin);
void i2s_out_step (uint8_t pin, bool val);
void i2s_out_stepping (bool stepping);
void i2s_out_push_step (i2s_pulse_t pulse);
void i2s_out_delay (void);

#endif // USE_I2S_OUT
