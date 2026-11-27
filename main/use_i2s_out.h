/*
  use_i2s_out.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2024-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#define USE_I2S_OUT 1

#include "i2s_out.h"

static inline int gpio_rll_get_level (gpio_dev_t *hw, gpio_num_t gpio_num)
{
    static const DRAM_ATTR uint32_t trettito = 32;

    if (gpio_num < trettito) {
        return (hw->in >> gpio_num) & 0x1;
    } else {
        return (HAL_FORCE_READ_U32_REG_FIELD(hw->in1, data) >> (gpio_num - trettito)) & 0x1;
    }
}

#define DIGITAL_IN(pin) (pin >= I2S_OUT_PIN_BASE ? i2s_out_state(pin - I2S_OUT_PIN_BASE) : gpio_rll_get_level(&GPIO, pin))
#define DIGITAL_OUT(pin, state) { if(pin >= I2S_OUT_PIN_BASE) i2s_out_write(pin - I2S_OUT_PIN_BASE, state); else gpio_ll_set_level(&GPIO, pin, state); }
#define STEP_OUT(pin, state) { if(pin >= I2S_OUT_PIN_BASE) i2s_out_step(pin - I2S_OUT_PIN_BASE, state); else gpio_ll_set_level(&GPIO, pin, state); }

/**/
