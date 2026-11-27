/*
  i2s_out.c

  Part of grblHAL driver for ESP32

  Copyright (c) 2026 Terje Io

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

#include "driver.h"

#include "grbl/report.h"

#if USE_I2S_OUT

#include <driver/periph_ctrl.h>
#include <rom/lldesc.h>
#include <soc/i2s_struct.h>
#include "hal/i2s_ll.h"

#include "i2s_out.h"

typedef struct {
    int32_t step_samples;
    int32_t delay_samples;
} i2s_samples_t;

static struct {
    bool initialized;
    bool stepping;
    uint32_t min_samples;
    volatile uint32_t port_data;
    volatile uint32_t step_data;
    i2s_samples_t cfg;
} i2s_out = {0};

//
// Public API functions
//

void IRAM_ATTR i2s_out_delay (void)
{
    while(!I2S0.state.tx_idle);
}

void IRAM_ATTR i2s_out_write (uint8_t pin, bool val)
{
    static uint32_t buf = 0xFFFFFFFF;

    if(val)
        i2s_out.port_data |= bit(pin);
    else
        i2s_out.port_data &= ~bit(pin);

    if(i2s_out.port_data != buf) {
        while(i2s_out.stepping && I2S0.int_raw.tx_wfull);
        I2S0.fifo_wr = (buf = i2s_out.port_data);
    }
}

bool IRAM_ATTR i2s_out_state (uint8_t pin)
{
    return (!!(i2s_out.port_data & bit(pin)));
}

void IRAM_ATTR i2s_out_step (uint8_t pin, bool val)
{
    if(val)
        i2s_out.step_data |= bit(pin);
    else
        i2s_out.step_data &= ~bit(pin);
}

void IRAM_ATTR i2s_out_push_step (i2s_pulse_t pulse)
{
    uint32_t step_data;
    i2s_samples_t num = {0};

    while(!I2S0.int_raw.tx_rempty);

    if(pulse.dir_changed)
        num.delay_samples = i2s_out.cfg.delay_samples;
    if(pulse.step) {
        step_data = i2s_out.step_data | i2s_out.port_data;
        num.step_samples = i2s_out.cfg.step_samples;
    }

    while(!I2S0.int_raw.tx_wfull) {
        if(num.delay_samples) {
            I2S0.fifo_wr = i2s_out.port_data;
            if(--num.delay_samples == 0 && num.step_samples == 0)
                break;
        } else if(num.step_samples) {
            I2S0.fifo_wr = step_data;
            num.step_samples--;
        } else {
            I2S0.fifo_wr = i2s_out.port_data;
            break;
        }
    }
}

IRAM_ATTR void i2s_out_stepping (bool stepping)
{
    i2s_out.stepping = stepping;
}

uint32_t i2s_out_init (settings_t *settings)
{
    int32_t toff_samples = STEP_PULSE_TOFF_MIN > (float)I2S_OUT_USEC_PER_PULSE ? i2s_sample_cfg(STEP_PULSE_TOFF_MIN) : 1;
    i2s_out.cfg.delay_samples = i2s_sample_cfg(settings->steppers.pulse_delay_microseconds);
    i2s_out.cfg.step_samples = i2s_sample_cfg(settings->steppers.pulse_microseconds);
    i2s_out.min_samples = i2s_out.cfg.step_samples + toff_samples;

    if(i2s_out.cfg.delay_samples + i2s_out.cfg.step_samples + toff_samples > I2S_TX_DATA_NUM)
        i2s_out.cfg.delay_samples = I2S_TX_DATA_NUM - toff_samples - i2s_out.cfg.step_samples;

    if(!i2s_out.initialized) {

        // To make sure hardware is enabled before any hardware register operations.
        periph_module_reset(PERIPH_I2S0_MODULE);
        periph_module_enable(PERIPH_I2S0_MODULE);

        i2s_ll_tx_stop(&I2S0);
        i2s_ll_rx_stop(&I2S0);
        i2s_ll_tx_force_enable_fifo_mod(&I2S0, true);
        i2s_ll_tx_enable_pdm(&I2S0, false);
        i2s_ll_enable_dma(&I2S0, false);
        i2s_ll_enable_lcd(&I2S0, false);

        i2s_ll_tx_set_chan_mod(&I2S0, I2S_CHANNEL_FMT_RIGHT_LEFT);
#if I2S_OUT_NUM_BITS == 16
        i2s_ll_tx_set_sample_bit(&I2S0, I2S_BITS_PER_SAMPLE_16BIT, I2S_BITS_PER_SAMPLE_16BIT);
#else
        i2s_ll_tx_set_sample_bit(&I2S0, I2S_BITS_PER_SAMPLE_32BIT, I2S_BITS_PER_SAMPLE_16BIT);
#endif
        i2s_ll_tx_enable_mono_mode(&I2S0, false);
        i2s_ll_tx_enable_msb_right(&I2S0, true);
        i2s_ll_tx_set_ws_width(&I2S0, 0);
        i2s_ll_tx_enable_msb_shift(&I2S0, false);

        i2s_ll_tx_clk_set_src(&I2S0, I2S_CLK_D2CLK); // 160 MHz

//      fs = 160MHz / (mclk_div + b / a) / 2 / I2S_OUT_NUM_BITS

#if I2S_OUT_NUM_BITS == 16
  #if I2S_OUT_USEC_PER_PULSE == 1
        i2s_ll_mclk_div_t clk = { .mclk_div = 5, .a = 0, .b = 0 };  // 16MHz
  #elif I2S_OUT_USEC_PER_PULSE == 2
        i2s_ll_mclk_div_t clk = { .mclk_div = 10, .a = 0, .b = 0 }; // 8Mhz
  #elif I2S_OUT_USEC_PER_PULSE == 4
        i2s_ll_mclk_div_t clk = { .mclk_div = 20, .a = 0, .b = 0 }; // 4Mhz
  #endif
#else // 32bit
  #if I2S_OUT_USEC_PER_PULSE == 1
        i2s_ll_mclk_div_t clk = { .mclk_div = 2, .a = 10, .b = 5 }; // 32MHz
  #elif I2S_OUT_USEC_PER_PULSE == 2
        i2s_ll_mclk_div_t clk = { .mclk_div = 5, .a = 0, .b = 0 };  // 16Mhz
  #elif I2S_OUT_USEC_PER_PULSE == 4
        i2s_ll_mclk_div_t clk = { .mclk_div = 10, .a = 0, .b = 0 }; // 8Mhz
  #endif
#endif

        i2s_ll_tx_set_clk(&I2S0, &clk);
        i2s_ll_tx_set_bck_div_num(&I2S0, 2);

        i2s_out.initialized = true;

        // Route the i2s pins to the appropriate GPIO
        i2s_gpio_cfg(I2S_OUT_DATA, I2S0O_DATA_OUT23_IDX);
        i2s_gpio_cfg(I2S_OUT_BCK, I2S0O_BCK_OUT_IDX);
        i2s_gpio_cfg(I2S_OUT_WS, I2S0O_WS_OUT_IDX);

        i2s_ll_rx_reset(&I2S0);
        i2s_ll_tx_reset(&I2S0);
        i2s_ll_rx_reset_fifo(&I2S0);
        i2s_ll_tx_reset_fifo(&I2S0);

        i2s_ll_tx_stop_on_fifo_empty(&I2S0, true);
        i2s_ll_tx_start(&I2S0);

        ets_delay_us(20);
        i2s_ll_tx_stop_on_fifo_empty(&I2S0, false);
    }

    return i2s_out.initialized ? i2s_out.min_samples * I2S_OUT_USEC_PER_PULSE : 0;
}

#endif // USE_I2S_OUT
