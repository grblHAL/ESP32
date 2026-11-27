/*
  i2s_out_s3.c

  Part of grblHAL driver for ESP32-S3

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

// NOTES:
// Only 32 bit mode supported.
// Only 2 and 4 us sample length supported.

#include "driver.h"

#if USE_I2S_OUT

#include <driver/periph_ctrl.h>
#include <rom/lldesc.h>
#include <soc/i2s_struct.h>
#include <soc/gdma_struct.h>
#include <soc/gdma_channel.h>
#include "hal/i2s_ll.h"
#include "hal/gdma_ll.h"
#include "hal/dma_types.h"
#include "soc/gdma_periph.h"
#include "soc/system_reg.h"

#include "i2s_out.h"

#define DMA_SAMPLE_COUNT 64 /* number of samples per DMA buffer */
#define I2S_OUT_DMABUF_COUNT 5  /* number of DMA buffers to store data */
#define I2S_OUT_DMABUF_LEN 256 /* maximum size in bytes (4092 is DMA's limit) */

typedef struct i2s_buffer {
    volatile uint_fast16_t idx;
    dma_descriptor_t dma;
    uint32_t samples[DMA_SAMPLE_COUNT];
    struct i2s_buffer *next;
} i2s_buffer_t;

typedef struct {
    i2s_buffer_t **buffers;
    i2s_buffer_t *current;
    int32_t channel;
} i2s_out_dma_t;

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
    dma_descriptor_t *prev;
    i2s_out_dma_t dma;
} i2s_out = {0};

// Hack, to be replaced with framework call?
static int32_t allocate_dma_channel (void)
{
    uint32_t ch = SOC_GDMA_PAIRS_PER_GROUP;

    do {
       if(GDMA.channel[--ch].out.link.addr == 0)
           return ch;
    } while(ch);

    return -1;
}

static inline __attribute__((always_inline)) void start_dma (dma_descriptor_t *desc)
{
    while(i2s_out.prev->dw0.owner);
    while(!I2S0.state.tx_idle);
    i2s_out.prev = desc;
    desc->dw0.owner = 1;

    i2s_ll_tx_reset(&I2S0);
    gdma_ll_tx_set_desc_addr(&GDMA, i2s_out.dma.channel, (uint32_t)desc);
    gdma_ll_tx_start(&GDMA, i2s_out.dma.channel);
}

//
// Public API functions
//

void IRAM_ATTR i2s_out_delay (void)
{
    while(i2s_out.prev->dw0.owner);
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

        if(i2s_out.stepping)
            buf = i2s_out.port_data;
        else {
            i2s_buffer_t *fifo = i2s_out.dma.current;

            fifo->samples[0] = (buf = i2s_out.port_data);
            fifo->dma.dw0.length = 4;

            (i2s_out.dma.current = fifo->next)->idx = 0;

            start_dma(&fifo->dma);
        }
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
    i2s_buffer_t *fifo = i2s_out.dma.current;

    uint32_t step_data;
    i2s_samples_t num = {0};

    if(pulse.dir_changed)
        num.delay_samples = i2s_out.cfg.delay_samples;
    if(pulse.step) {
        step_data = i2s_out.step_data | i2s_out.port_data;
        num.step_samples = i2s_out.cfg.step_samples;
    }

    while(fifo->idx < DMA_SAMPLE_COUNT) {
        if(num.delay_samples) {
            fifo->samples[fifo->idx++] = i2s_out.port_data;
            if(--num.delay_samples == 0 && num.step_samples == 0)
                break;
        } else if(num.step_samples) {
            fifo->samples[fifo->idx++] = step_data;
            num.step_samples--;
        } else {
            fifo->samples[fifo->idx++] = i2s_out.port_data;
            break;
        }
    }

    fifo->dma.dw0.length = fifo->idx << 2; // * 4

    (i2s_out.dma.current = fifo->next)->idx = 0;

    start_dma(&fifo->dma);
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

    if(i2s_out.cfg.delay_samples + i2s_out.cfg.step_samples + toff_samples > DMA_SAMPLE_COUNT)
        i2s_out.cfg.delay_samples = DMA_SAMPLE_COUNT - toff_samples - i2s_out.cfg.step_samples;

    if(!i2s_out.initialized) {

        if((i2s_out.dma.channel = allocate_dma_channel()) == -1)
            return 0;

        // To make sure hardware is enabled before any hardware register operations.
        periph_module_reset(PERIPH_I2S0_MODULE);
        periph_module_enable(PERIPH_I2S0_MODULE);
    //    periph_module_reset(PERIPH_GDMA_MODULE);
    //    periph_module_enable(PERIPH_GDMA_MODULE);

        if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0) {
            REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
            REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
            REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
            REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        }

        // Route the i2s pins to the appropriate GPIO
        i2s_gpio_cfg(I2S_OUT_DATA, I2S0O_SD_OUT_IDX);
        i2s_gpio_cfg(I2S_OUT_BCK, I2S0O_BCK_OUT_IDX);
        i2s_gpio_cfg(I2S_OUT_WS, I2S0O_WS_OUT_IDX);

        // Allocate the array of pointers to the buffers
        if((i2s_out.dma.buffers = (i2s_buffer_t **)malloc(sizeof(i2s_buffer_t *) * I2S_OUT_DMABUF_COUNT)) == NULL)
            return false;

        // Allocate each buffer that can be used by the DMA controller
        for(int i = 0; i < I2S_OUT_DMABUF_COUNT; i++) {
            if((i2s_out.dma.buffers[i] = (i2s_buffer_t *)heap_caps_calloc(1, sizeof(i2s_buffer_t), MALLOC_CAP_DMA)) == NULL)
                return false;
        }

        for(int i = 0; i < I2S_OUT_DMABUF_COUNT; i++) {
            i2s_out.dma.buffers[i]->dma.dw0.suc_eof = 1;
            i2s_out.dma.buffers[i]->dma.dw0.size = DMA_SAMPLE_COUNT / sizeof(uint32_t);
            i2s_out.dma.buffers[i]->dma.buffer = i2s_out.dma.buffers[i]->samples;
            i2s_out.dma.buffers[i]->next = ((i < (I2S_OUT_DMABUF_COUNT - 1)) ? (i2s_out.dma.buffers[i + 1]) : i2s_out.dma.buffers[0]);
        }

        i2s_out.initialized = true;
        i2s_out.dma.current = i2s_out.dma.buffers[0];
        i2s_out.prev = &i2s_out.dma.current->dma;

        i2s_ll_tx_stop(&I2S0);

  //    gdma_ahb_hal_connect_peri(&GDMA, i2s_out.dma.channel, GDMA_CHANNEL_DIRECTION_TX, GDMA_TRIG_PERIPH_I2S, SOC_GDMA_TRIG_PERIPH_I2S0);

        gdma_ll_enable_clock(&GDMA, 1);
        gdma_ll_tx_reset_channel(&GDMA, i2s_out.dma.channel);
        gdma_ll_tx_enable_interrupt(&GDMA, i2s_out.dma.channel, GDMA_LL_TX_EVENT_MASK, false);
        gdma_ll_tx_clear_interrupt_status(&GDMA, i2s_out.dma.channel, GDMA_LL_TX_EVENT_MASK);
        gdma_ll_tx_connect_to_periph(&GDMA, i2s_out.dma.channel, SOC_GDMA_TRIG_PERIPH_I2S0);
        gdma_ll_tx_set_eof_mode(&GDMA, i2s_out.dma.channel, 0); // Not needed?
        gdma_ll_tx_enable_auto_write_back(&GDMA, i2s_out.dma.channel, true);

#if I2S_OUT_NUM_BITS == 16

        // TODO
        I2S0.fifo_conf.tx_fifo_mod        = 0;   // 0: 16-bit dual channel data, 3: 32-bit single channel data
        I2S0.fifo_conf.rx_fifo_mod        = 0;   // 0: 16-bit dual channel data, 3: 32-bit single channel data
        I2S0.sample_rate_conf.tx_bits_mod = 16;  // default is 16-bits
        I2S0.sample_rate_conf.rx_bits_mod = 16;  // default is 16-bits

#else

  #if I2S_OUT_USEC_PER_PULSE == 2
        i2s_ll_mclk_div_t _clk = { .mclk_div = 2, .a = 20, .b = 10 }; // 16Mhz
  #elif I2S_OUT_USEC_PER_PULSE == 4
        i2s_ll_mclk_div_t _clk = { .mclk_div = 5, .a = 0, .b = 0 };   // 8Mhz
  #endif

        i2s_ll_tx_enable_pdm(&I2S0, false); // Enables TDM
        i2s_ll_rx_set_active_chan_mask(&I2S0, 1);
        i2s_ll_tx_enable_msb_shift(&I2S0, 0);
        i2s_ll_tx_set_sample_bit(&I2S0, 32, 32); // ?
        i2s_ll_tx_set_ws_width(&I2S0, 1);
     // I2S0.tx_timing.tx_bck_out_dm = 1;
        I2S0.tx_timing.tx_ws_out_dm = 1;

#endif

        i2s_ll_tx_set_ws_idle_pol(&I2S0, 1);
        i2s_ll_tx_clk_set_src(&I2S0, I2S_CLK_D2CLK); // 160 MHz
        i2s_ll_tx_set_clk(&I2S0, &_clk);
        i2s_ll_tx_set_bck_div_num(&I2S0, 4);
        i2s_ll_tx_enable_clock(&I2S0);
        i2s_ll_tx_reset(&I2S0);
        i2s_ll_tx_reset_fifo(&I2S0);
        I2S0.tx_conf.tx_stop_en = 1;

        gdma_ll_tx_stop(&GDMA, i2s_out.dma.channel);

        i2s_ll_tx_reset(&I2S0);
        i2s_ll_tx_reset_fifo(&I2S0);

        gdma_ll_tx_reset_channel(&GDMA, i2s_out.dma.channel);
        gdma_ll_tx_set_desc_addr(&GDMA, i2s_out.dma.channel, (uint32_t)&i2s_out.dma.current->dma);
    //    gdma_ll_tx_enable_data_burst(&GDMA, i2s_out.dma.channel, true);
    //    gdma_ll_tx_enable_descriptor_burst(&GDMA, i2s_out.dma.channel, true);

        I2S0.tx_conf.tx_update = 1;
        while(I2S0.tx_conf.tx_update);
    //    i2s_ll_tx_start(&I2S0); // Fails?
        gdma_ll_tx_start(&GDMA, i2s_out.dma.channel);
        I2S0.tx_conf.tx_start = 1;

        i2s_out_write(0, false);
    }

    return i2s_out.initialized ? i2s_out.min_samples * I2S_OUT_USEC_PER_PULSE : 0;
}

#endif // USE_I2S_OUT
