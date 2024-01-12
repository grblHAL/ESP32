/*
    i2s_out_s3.c

    Part of Grbl_ESP32 and grblHAL

    Basic GPIO expander using the ESP32 I2S peripheral (output)

    2020    - Michiyasu Odaki
    2024    - Terje Io

    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// !!! Work in progress, not yet functional !!!

#include "driver.h"

#include "grbl/report.h"

#if USE_I2S_OUT

//#include <FreeRTOS.h>
#include <driver/periph_ctrl.h>
#include <rom/lldesc.h>
#include <soc/i2s_struct.h>
#include <soc/gdma_struct.h>
#include <soc/gdma_channel.h>
#include "hal/i2s_ll.h"
#include "hal/gdma_ll.h"
#include "hal/dma_types.h"
#include "hal/interrupt_controller_hal.h"
#include <freertos/queue.h>
#include "esp_intr_alloc.h"
#include "soc/gdma_periph.h"
#include "soc/system_reg.h"
#include <stdatomic.h>

#include "i2s_out.h"

#define delay(ms) hal.delay_ms(ms, 0);

//
// Configrations for DMA connected I2S
//
// One DMA buffer transfer takes about 2 ms
//   I2S_OUT_DMABUF_LEN / I2S_SAMPLE_SIZE x I2S_OUT_USEC_PER_PULSE
//   = 2000 / 4 x 4
//   = 2000us = 2ms
// If I2S_OUT_DMABUF_COUNT is 5, it will take about 10 ms for all the DMA buffer transfers to finish.
//
// Increasing I2S_OUT_DMABUF_COUNT has the effect of preventing buffer underflow,
// but on the other hand, it leads to a delay with pulse and/or non-pulse-generated I/Os.
// The number of I2S_OUT_DMABUF_COUNT should be chosen carefully.
//
// Reference information:
//   FreeRTOS task time slice = portTICK_PERIOD_MS = 1 ms (ESP32 FreeRTOS port)
//
#define I2S_SAMPLE_SIZE 4                                       /* 4 bytes, 32 bits per sample */
#define DMA_SAMPLE_COUNT (I2S_OUT_DMABUF_LEN / I2S_SAMPLE_SIZE) /* number of samples per buffer */
#define SAMPLE_SAFE_COUNT (20 / I2S_OUT_USEC_PER_PULSE)         /* prevent buffer overrun (GRBL's $0 should be less than or equal 20) */
#ifndef I2S_OUT_INIT_VAL
#define I2S_OUT_INIT_VAL 0
#endif
#define I2S_OUT_DETACH_PORT_IDX 0x100
#define I2S_LOCAL_QUEUE 0 // Set 0 for FreeRTOS queue, 8 or 16 for local queue

typedef struct {
    uint32_t **buffers;
    uint32_t *current;
    uint32_t rw_pos;
    dma_descriptor_t **desc;
    int32_t channel;
    intr_handle_t intr_handle;
#if !I2S_LOCAL_QUEUE
    xQueueHandle queue;
#endif
} i2s_out_dma_t;

typedef struct {
    bool initialized;
    atomic_uint_least32_t port_data; // output value
    volatile uint32_t pulse_period;
    uint32_t remain_time_until_next_pulse;  // Time remaining until the next pulse (usec)
    gpio_num_t ws_pin;
    gpio_num_t bck_pin;
    gpio_num_t data_pin;
    volatile i2s_out_pulse_func_t pulse_func;
    volatile i2s_out_pulser_status_t pulser_status;
    portMUX_TYPE spinlock, pulser_spinlock;
    i2s_out_dma_t dma;
} i2s_sr_t;

typedef struct {
    volatile uint_fast16_t head;
    volatile uint_fast16_t tail;
    dma_descriptor_t *descr[I2S_LOCAL_QUEUE];
} i2s_dma_queue_t;

static const DRAM_ATTR uint32_t i2s_tx_int_flags = GDMA_LL_EVENT_TX_DONE|GDMA_LL_EVENT_TX_TOTAL_EOF;

static uint32_t pd = 0;

#if I2S_LOCAL_QUEUE
static i2s_dma_queue_t dma_queue = {0};
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
#endif

static i2s_sr_t i2s_sr = {
    .ws_pin   = 255,
    .bck_pin  = 255,
    .data_pin = 255,
    .pulser_status = STOPPED,
    .port_data = ATOMIC_VAR_INIT(0),
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
    .pulser_spinlock = portMUX_INITIALIZER_UNLOCKED
};

// inner lock
#define I2S_OUT_ENTER_CRITICAL()                        \
    do {                                                \
        if (xPortInIsrContext()) {                      \
            portENTER_CRITICAL_ISR(&i2s_sr.spinlock);   \
        } else {                                        \
            portENTER_CRITICAL(&i2s_sr.spinlock);       \
        }                                               \
    } while (0)
#define I2S_OUT_EXIT_CRITICAL()                         \
    do {                                                \
        if (xPortInIsrContext()) {                      \
            portEXIT_CRITICAL_ISR(&i2s_sr.spinlock);    \
        } else {                                        \
            portEXIT_CRITICAL(&i2s_sr.spinlock);        \
        }                                               \
    } while (0)

#define I2S_OUT_ENTER_CRITICAL_ISR() portENTER_CRITICAL_ISR(&i2s_sr.spinlock)
#define I2S_OUT_EXIT_CRITICAL_ISR() portEXIT_CRITICAL_ISR(&i2s_sr.spinlock)

// outer lock
#define I2S_OUT_PULSER_ENTER_CRITICAL()                         \
    do {                                                        \
        if (xPortInIsrContext()) {                              \
            portENTER_CRITICAL_ISR(&i2s_sr.pulser_spinlock);    \
        } else {                                                \
            portENTER_CRITICAL(&i2s_sr.pulser_spinlock);        \
        }                                                       \
    } while (0)
#define I2S_OUT_PULSER_EXIT_CRITICAL()                          \
    do {                                                        \
        if (xPortInIsrContext()) {                              \
            portEXIT_CRITICAL_ISR(&i2s_sr.pulser_spinlock);     \
        } else {                                                \
            portEXIT_CRITICAL(&i2s_sr.pulser_spinlock);         \
        }                                                       \
    } while (0)

#define I2S_OUT_PULSER_ENTER_CRITICAL_ISR() portENTER_CRITICAL_ISR(&i2s_sr.pulser_spinlock)
#define I2S_OUT_PULSER_EXIT_CRITICAL_ISR() portEXIT_CRITICAL_ISR(&i2s_sr.pulser_spinlock)

//
// Internal functions
//
static inline void gpio_matrix_out_check (uint8_t gpio, uint32_t signal_idx, bool out_inv, bool oen_inv)
{
    if (gpio != 255) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
        gpio_set_direction((gpio_num_t)gpio, GPIO_MODE_OUTPUT);
        gpio_matrix_out(gpio, signal_idx, out_inv, oen_inv);
    }
}

static inline void i2s_out_single_data (void)
{
#if I2S_OUT_NUM_BITS == 16
    uint32_t port_data = atomic_load(&i2s_sr.port_data);
    port_data <<= 16;                   // Shift needed. This specification is not spelled out in the manual.
    I2S0.conf_single_data = port_data;  // Apply port data in real-time (static I2S)
#else
    I2S0.conf_single_data = atomic_load(&i2s_sr.port_data);  // Apply port data in real-time (static I2S)
#endif
}

static inline void IRAM_ATTR i2s_clear_dma_buffer (dma_descriptor_t *dma_desc, uint32_t port_data)
{
    uint32_t *buf = (uint32_t *)dma_desc->buffer, i = DMA_SAMPLE_COUNT;

    do {
        *buf++ = port_data;
    } while(--i);
    // Restore the buffer length.
    // The length may have been changed short when the data was filled in to prevent buffer overrun.
    dma_desc->dw0.length = I2S_OUT_DMABUF_LEN;
}

static void IRAM_ATTR i2s_clear_o_dma_buffers (uint32_t port_data)
{
    for(int i = 0; i < I2S_OUT_DMABUF_COUNT; i++) {

        // Initialize DMA descriptor
        memset(i2s_sr.dma.desc[i], 0, sizeof(dma_descriptor_t));

        i2s_sr.dma.desc[i]->dw0.owner = 1;
        i2s_sr.dma.desc[i]->dw0.suc_eof = 1;
        i2s_sr.dma.desc[i]->dw0.length = I2S_OUT_DMABUF_LEN;
        i2s_sr.dma.desc[i]->dw0.size = I2S_OUT_DMABUF_LEN;
        i2s_sr.dma.desc[i]->buffer = i2s_sr.dma.buffers[i];
        i2s_sr.dma.desc[i]->next = (dma_descriptor_t *)((i < (I2S_OUT_DMABUF_COUNT - 1)) ? (i2s_sr.dma.desc[i + 1]) : i2s_sr.dma.desc[0]);

        i2s_clear_dma_buffer(i2s_sr.dma.desc[i], port_data);
    }
}

static void IRAM_ATTR i2s_out_gpio_attach (uint8_t ws, uint8_t bck, uint8_t data)
{
    // Route the i2s pins to the appropriate GPIO
    gpio_matrix_out_check(data, I2S0O_SD_OUT_IDX, false, false);
    gpio_matrix_out_check(bck, I2S0O_BCK_OUT_IDX, false, false);
    gpio_matrix_out_check(ws, I2S0O_WS_OUT_IDX, false, false);
}

static void IRAM_ATTR i2s_out_gpio_detach (uint8_t ws, uint8_t bck, uint8_t data)
{
    // Route the i2s pins to the appropriate GPIO
    gpio_matrix_out_check(ws, I2S_OUT_DETACH_PORT_IDX, false, false);
    gpio_matrix_out_check(bck, I2S_OUT_DETACH_PORT_IDX, false, false);
    gpio_matrix_out_check(data, I2S_OUT_DETACH_PORT_IDX, false, false);
}

static void IRAM_ATTR i2s_out_gpio_shiftout (uint32_t port_data)
{
    uint32_t i = I2S_OUT_NUM_BITS;

    gpio_set_level(i2s_sr.ws_pin, 0);

    do {
        gpio_set_level(i2s_sr.data_pin, !!(port_data & bit(--i)));
        gpio_set_level(i2s_sr.bck_pin, 1);
        gpio_set_level(i2s_sr.bck_pin, 0);
    } while(i);

//    gpio_set_level(i2s_sr.data_pin, 0);
    gpio_set_level(i2s_sr.ws_pin, 1);  // Latch
    gpio_set_level(i2s_sr.ws_pin, 0);
}

static void IRAM_ATTR i2s_out_stop (void)
{
    hal.stream.write("stop" ASCII_EOL);

    I2S_OUT_ENTER_CRITICAL();

    // Stop FIFO DMA
    gdma_ll_tx_stop(&GDMA, i2s_sr.dma.channel);
    gdma_ll_tx_enable_interrupt(&GDMA, i2s_sr.dma.channel, i2s_tx_int_flags, i2s_sr.pulser_status != PASSTHROUGH);

//!    i2s_ll_tx_stop_link(&I2S0);
//    I2S0.out_link.stop = 1;

    // Disconnect DMA from FIFO
//!    i2s_ll_enable_dma(&I2S0);
//    I2S0.fifo_conf.dscr_en = 0;  //Unset this bit to disable I2S DMA mode. (R/W)

    // stop TX module
    i2s_ll_tx_stop(&I2S0);

    // Force WS to LOW before detach
    // This operation prevents unintended WS edge trigger when detach
    gpio_set_level(i2s_sr.ws_pin, 0);

    // Now, detach GPIO pin from I2S
    i2s_out_gpio_detach(i2s_sr.ws_pin, i2s_sr.bck_pin, i2s_sr.data_pin);

    // Force BCK to LOW
    // After the TX module is stopped, BCK always seems to be in LOW.
    // However, I'm going to do it manually to ensure the BCK's LOW.
    gpio_set_level(i2s_sr.bck_pin, 0);

    // Transmit recovery data to 74HC595
    uint32_t port_data = atomic_load(&i2s_sr.port_data);  // current expanded port value
    i2s_out_gpio_shiftout(port_data);

    //clear pending interrupt
    gdma_ll_tx_clear_interrupt_status(&GDMA, i2s_sr.dma.channel, gdma_ll_tx_get_interrupt_status(&GDMA, i2s_sr.dma.channel));

    I2S_OUT_EXIT_CRITICAL();
}

static bool IRAM_ATTR i2s_out_start (i2s_out_pulser_status_t pulser_status)
{
    static dma_descriptor_t pass_dma = {
        .dw0.owner = 1,
        .dw0.suc_eof = 0,
        .dw0.length = sizeof(i2s_sr.port_data),
        .dw0.size = sizeof(i2s_sr.port_data),
        .buffer = &pd, // &i2s_sr.port_data,
        .next = &pass_dma
    };

    if(!i2s_sr.initialized)
        return false;

    I2S_OUT_ENTER_CRITICAL();

    if(i2s_sr.pulse_func == NULL)
        pulser_status = PASSTHROUGH;

    if(i2s_sr.pulser_status == pulser_status) {
        I2S_OUT_EXIT_CRITICAL();
        return true;
    }
    gpio_set_level(41, 0);

    gdma_ll_tx_stop(&GDMA, i2s_sr.dma.channel);

    //start DMA link
    i2s_ll_tx_reset(&I2S0);
    i2s_ll_tx_reset_fifo(&I2S0);
//    i2s_out_gpio_attach(i2s_sr.ws_pin, i2s_sr.bck_pin, i2s_sr.data_pin);

    // Transmit recovery data to 74HC595
    uint32_t port_data = atomic_load(&i2s_sr.port_data);  // current expanded port value
//    i2s_out_gpio_shiftout(port_data);

    if((i2s_sr.pulser_status = pulser_status) == PASSTHROUGH)
        pd = port_data;
    else {
        i2s_clear_o_dma_buffers(port_data);
#if I2S_LOCAL_QUEUE
        dma_queue.tail = dma_queue.head;
#endif
        i2s_sr.dma.rw_pos = 0;
        i2s_sr.dma.current = i2s_sr.dma.desc[0]->buffer;
    }

    gdma_ll_tx_reset_channel(&GDMA, i2s_sr.dma.channel);
    gdma_ll_tx_set_desc_addr(&GDMA, i2s_sr.dma.channel, (uint32_t)(i2s_sr.pulser_status == PASSTHROUGH ? &pass_dma : i2s_sr.dma.desc[0]));
//    gdma_ll_tx_connect_to_periph(&GDMA, i2s_sr.dma.channel, GDMA_TRIG_PERIPH_I2S, SOC_GDMA_TRIG_PERIPH_I2S0);
//    gdma_ll_tx_set_eof_mode(&GDMA, i2s_sr.dma.channel, 0); // Not needed?
//    gdma_ll_tx_enable_data_burst(&GDMA, i2s_sr.dma.channel, true);
//    gdma_ll_tx_enable_descriptor_burst(&GDMA, i2s_sr.dma.channel, true);
    gdma_ll_tx_clear_interrupt_status(&GDMA, i2s_sr.dma.channel, gdma_ll_tx_get_interrupt_status(&GDMA, i2s_sr.dma.channel));
    gdma_ll_tx_enable_interrupt(&GDMA, i2s_sr.dma.channel, i2s_tx_int_flags, i2s_sr.pulser_status != PASSTHROUGH);

//    I2S0.tx_conf.tx_stop_en = 1;  // BCK and WCK are suppressed while FIFO is empty - no ll func!

    I2S0.tx_conf.tx_update = 1;
    while (I2S0.tx_conf.tx_update);

//    i2s_ll_tx_start(&I2S0);
    gdma_ll_tx_start(&GDMA, i2s_sr.dma.channel);

    I2S0.tx_conf.tx_start = 1;
/*
//    I2S0.tx_conf.tx_start = 1;
    // Wait for the first FIFO data to prevent the unintentional generation of 0 data
    ets_delay_us(20);

    I2S0.tx_conf.tx_stop_en = 1;  // BCK and WCK are suppressed while FIFO is empty - no ll func!
*/
    gpio_set_level(41, 1);

    I2S_OUT_EXIT_CRITICAL();

    return true;
}

//
// I2S out DMA Interrupts handler
//
static void IRAM_ATTR i2s_out_intr_handler (void *arg)
{
    portBASE_TYPE high_priority_task_awoken = pdFALSE;

    uint32_t irq = gdma_ll_tx_get_interrupt_status(&GDMA, i2s_sr.dma.channel);

//    gpio_set_level(41, 1);

    if(irq & i2s_tx_int_flags) {

        // Get the descriptor of the last item in the linked list
        dma_descriptor_t *finish_desc = (dma_descriptor_t *)gdma_ll_tx_get_eof_desc_addr(&GDMA, i2s_sr.dma.channel);

        // Finished stepping?
        if(irq & GDMA_LL_EVENT_TX_TOTAL_EOF) {

            i2s_out_start(PASSTHROUGH);
        }

        // If the queue is full it's because we have an underflow,
        // more than buf_count isr without new data, remove the front buffer
        else if((irq & GDMA_LL_EVENT_TX_DONE) && i2s_sr.pulser_status != PASSTHROUGH) {

//            gpio_set_level(41, 1);

            uint32_t port_data = 0;
            dma_descriptor_t *front_desc;

#if I2S_LOCAL_QUEUE

            I2S_OUT_PULSER_ENTER_CRITICAL_ISR();

            uint32_t qptr = (dma_queue.head + 1) & (I2S_LOCAL_QUEUE - 1);  // Get next head pointer

            if(dma_queue.tail == qptr) {

                front_desc = dma_queue.descr[dma_queue.tail++];


                if(i2s_sr.pulser_status == STEPPING)
                    port_data = atomic_load(&i2s_sr.port_data);


                i2s_clear_dma_buffer(front_desc, port_data);

                dma_queue.tail &= (I2S_LOCAL_QUEUE - 1);
            }

            // Send a DMA complete event to the I2S bitstreamer task with finished buffer
            if((dma_queue.descr[dma_queue.head] = finish_desc) > 100)
                dma_queue.head = qptr;

            I2S_OUT_PULSER_EXIT_CRITICAL_ISR();

#else

            if(xQueueIsQueueFullFromISR(i2s_sr.dma.queue)) {

                // Remove a descriptor from the DMA complete event queue
                xQueueReceiveFromISR(i2s_sr.dma.queue, &front_desc, &high_priority_task_awoken);

                I2S_OUT_PULSER_ENTER_CRITICAL_ISR();

                if(i2s_sr.pulser_status == STEPPING)
                    port_data = atomic_load(&i2s_sr.port_data);

                I2S_OUT_PULSER_EXIT_CRITICAL_ISR();

                i2s_clear_dma_buffer(front_desc, port_data);
            }

            // Send a DMA complete event to the I2S bitstreamer task with finished buffer
            xQueueSendFromISR(i2s_sr.dma.queue, &finish_desc, &high_priority_task_awoken);

#endif
//            gpio_set_level(41, 0);
        }
    }

    gdma_ll_tx_clear_interrupt_status(&GDMA, i2s_sr.dma.channel, 0xFF);
//    gpio_set_level(41, 0);

    if(high_priority_task_awoken == pdTRUE)
        portYIELD_FROM_ISR();
}

static void IRAM_ATTR i2s_fillout_dma_buffer (dma_descriptor_t *dma_desc)
{
    uint32_t *buf = (uint32_t *)dma_desc->buffer;

    // It reuses the oldest (just transferred) buffer with the name "current"
    // and fills the buffer for later DMA.

    i2s_sr.dma.rw_pos = 0;

    //
    // Fillout the buffer for pulse
    //
    // To avoid buffer overflow, all of the maximum pulse width (normally about 10us)
    // is adjusted to be in a single buffer.
    // DMA_SAMPLE_SAFE_COUNT is referred to as the margin value.
    // Therefore, if a buffer is close to full and it is time to generate a pulse,
    // the generation of the buffer is interrupted (the buffer length is shortened slightly)
    // and the pulse generation is postponed until the next buffer is filled.
    //
    while (i2s_sr.dma.rw_pos < (DMA_SAMPLE_COUNT - SAMPLE_SAFE_COUNT)) {

        // no data to read (buffer empty)
        if (i2s_sr.remain_time_until_next_pulse < I2S_OUT_USEC_PER_PULSE) {

            // pulser status may change in pulse phase func, so I need to check it every time.
            if (i2s_sr.pulser_status == STEPPING) {

                // fillout future DMA buffer (tail of the DMA buffer chains)

                uint32_t old_rw_pos = i2s_sr.dma.rw_pos, period;

                I2S_OUT_PULSER_EXIT_CRITICAL();   // Temporarily unlocked status lock as it may be locked in pulse callback.

                i2s_sr.pulse_func();              // Insert steps.

                I2S_OUT_PULSER_ENTER_CRITICAL();  // Lock again.

                period = I2S_OUT_USEC_PER_PULSE * (i2s_sr.dma.rw_pos - old_rw_pos);

                // Calculate pulse period.
                if(i2s_sr.pulse_period >= period)
                    i2s_sr.remain_time_until_next_pulse += i2s_sr.pulse_period - period;
                else // too fast!
                    i2s_sr.remain_time_until_next_pulse += I2S_OUT_USEC_PER_PULSE;

                if (i2s_sr.pulser_status == WAITING) {
                    // i2s_out_set_passthrough() has called from the pulse function.
                    // It needs to go into pass-through mode.
                    // This DMA descriptor must be a tail of the chain.
                    dma_desc->dw0.suc_eof = 1; //?
                    dma_desc->next = NULL;  // Cut the DMA descriptor ring. This allow us to identify the tail of the buffer.
                } else if (i2s_sr.pulser_status == PASSTHROUGH) {
                    // i2s_out_reset() has called during the execution of the pulse function.
                    // I2S has already in static mode, and buffers has cleared to zero.
                    // To prevent the pulse function from being called back,
                    // we assume that the buffer is already full.
                    i2s_sr.remain_time_until_next_pulse = 0;                 // There is no need to fill the current buffer.
                    i2s_sr.dma.rw_pos                   = DMA_SAMPLE_COUNT;  // The buffer is full.
                    break;
                }
                continue;
            }
        }
        // no pulse data in push buffer (pulse off or idle or callback is not defined)
        buf[i2s_sr.dma.rw_pos++] = atomic_load(&i2s_sr.port_data);
        if (i2s_sr.remain_time_until_next_pulse >= I2S_OUT_USEC_PER_PULSE) {
            i2s_sr.remain_time_until_next_pulse -= I2S_OUT_USEC_PER_PULSE;
        } else {
            i2s_sr.remain_time_until_next_pulse = 0;
        }
    }
    // set filled length to the DMA descriptor
    dma_desc->dw0.length = i2s_sr.dma.rw_pos * I2S_SAMPLE_SIZE;
}

//
// I2S bitstream generator task
//

static inline void i2s_step_gen (dma_descriptor_t *dma_desc)
{
    i2s_sr.dma.current = (uint32_t *)dma_desc->buffer;
    // It reuses the oldest (just transferred) buffer with the name "current"
    // and fills the buffer for later DMA.

    I2S_OUT_PULSER_ENTER_CRITICAL();  // Lock pulser status

    if (i2s_sr.pulser_status == STEPPING) {
        //
        // Fillout the buffer for pulse
        //
        // To avoid buffer overflow, all of the maximum pulse width (normally about 10us)
        // is adjusted to be in a single buffer.
        // DMA_SAMPLE_SAFE_COUNT is referred to as the margin value.
        // Therefore, if a buffer is close to full and it is time to generate a pulse,
        // the generation of the buffer is interrupted (the buffer length is shortened slightly)
        // and the pulse generation is postponed until the next buffer is filled.
        //
        i2s_fillout_dma_buffer(dma_desc);
        dma_desc->dw0.length = i2s_sr.dma.rw_pos * I2S_SAMPLE_SIZE;
    } else if (i2s_sr.pulser_status == WAITING) {
        if (dma_desc->next == NULL) {
            // Tail of the DMA descriptor found
            // I2S TX module has already stopped by ISR
            //i2s_out_stop();
            //i2s_clear_o_dma_buffers(0);  // 0 for static I2S control mode (right ch. data is always 0)
            // You need to set the status before calling i2s_out_start()
            // because the process in i2s_out_start() is different depending on the status.
            i2s_out_start(PASSTHROUGH);
        } else {
            // Processing a buffer slightly ahead of the tail buffer.
            // We don't need to fill up the buffer by port_data any more.
            i2s_clear_dma_buffer(dma_desc, 0);  // Essentially, no clearing is required. I'll make sure I know when I've written something.
            i2s_sr.dma.rw_pos = 0;              // If someone calls i2s_out_push_sample, make sure there is no buffer overflow
            dma_desc->next = NULL;              // Cut the DMA descriptor ring. This allow us to identify the tail of the buffer.
        }
    } else {
        // Stepper paused (passthrough state, static I2S control mode)
        // In the passthrough mode, there is no need to fill the buffer with port_data.
        i2s_clear_dma_buffer(dma_desc, 0);  // Essentially, no clearing is required. I'll make sure I know when I've written something.
        i2s_sr.dma.rw_pos = 0;              // If someone calls i2s_out_push_sample, make sure there is no buffer overflow
    }

    I2S_OUT_PULSER_EXIT_CRITICAL();  // Unlock pulser status
}

#if I2S_LOCAL_QUEUE

static inline void IRAM_ATTR i2s_poll (void)
{
    // Get a DMA complete event from I2S isr
    if(dma_queue.tail != dma_queue.head) {
        I2S_OUT_PULSER_ENTER_CRITICAL();  // Lock pulser status

        dma_descriptor_t *dma_desc = dma_queue.descr[dma_queue.tail];

        dma_queue.tail = (dma_queue.tail + 1) & (I2S_LOCAL_QUEUE - 1);
        I2S_OUT_PULSER_EXIT_CRITICAL();  // Lock pulser status

        i2s_step_gen(dma_desc);
    }
}

static void IRAM_ATTR i2s_poll_rt (sys_state_t state)
{
    on_execute_realtime(state);

    i2s_poll();
}

static void IRAM_ATTR i2s_poll_dly (sys_state_t state)
{
    on_execute_delay(state);

    i2s_poll();
}

#else

static void IRAM_ATTR i2sOutTask (void *parameter)
{
    dma_descriptor_t *dma_desc;

    while(true) {

        // Wait a DMA complete event from I2S isr
        // (Blocks until a DMA transfer has completed)
        xQueueReceive(i2s_sr.dma.queue, &dma_desc, portMAX_DELAY);
if((uint32_t)dma_desc > 1000)
        i2s_step_gen(dma_desc);
    }
}

#endif

//
// External funtions
//
void IRAM_ATTR i2s_out_delay (void)
{
    I2S_OUT_PULSER_ENTER_CRITICAL();

    if (i2s_sr.pulser_status == PASSTHROUGH) {
        // Depending on the timing, it may not be reflected immediately,
        // so wait twice as long just in case.
        ets_delay_us(I2S_OUT_USEC_PER_PULSE * 2);
    } else {
        // Just wait until the data now registered in the DMA descripter
        // is reflected in the I2S TX module via FIFO.
        delay(I2S_OUT_DELAY_MS);
    }

    I2S_OUT_PULSER_EXIT_CRITICAL();
}

void IRAM_ATTR i2s_out_write (uint8_t pin, uint8_t val)
{
    if(val)
        atomic_fetch_or(&i2s_sr.port_data, bit(pin));
    else
        atomic_fetch_and(&i2s_sr.port_data, ~bit(pin));

    // It needs a lock for access, but I've given up because I need speed.
    // This is not a problem as long as there is no overlap between the status change and digitalWrite().
//    if(i2s_sr.pulser_status == PASSTHROUGH)
//        i2s_out_single_data();

    pd = atomic_load(&i2s_sr.port_data);
}

bool IRAM_ATTR i2s_out_state (uint8_t pin)
{
    uint32_t port_data = atomic_load(&i2s_sr.port_data);

    return !!(port_data & bit(pin));
}

uint32_t IRAM_ATTR i2s_out_push_sample (uint32_t num)
{
    if(num > SAMPLE_SAFE_COUNT)
        return 0;

    uint32_t n = num ? num : 1, port_data = atomic_load(&i2s_sr.port_data);

    // push at least one sample (even if num is zero)
    do {
        i2s_sr.dma.current[i2s_sr.dma.rw_pos++] = port_data;
    } while (--n);

    return num;
}

i2s_out_pulser_status_t IRAM_ATTR i2s_out_get_pulser_status (void)
{
    I2S_OUT_PULSER_ENTER_CRITICAL();

    i2s_out_pulser_status_t s = i2s_sr.pulser_status;

    I2S_OUT_PULSER_EXIT_CRITICAL();

    return s;
}

void IRAM_ATTR i2s_out_set_passthrough (void)
{
    I2S_OUT_PULSER_ENTER_CRITICAL();

    if (i2s_sr.pulser_status == STEPPING) {
        i2s_sr.pulser_status = WAITING;  // Start stopping the pulser
        delay(I2S_OUT_DELAY_MS);
    }

    I2S_OUT_PULSER_EXIT_CRITICAL();
}

void IRAM_ATTR i2s_out_set_stepping (void)
{
    I2S_OUT_PULSER_ENTER_CRITICAL();

    if (i2s_sr.pulser_status == STEPPING) {
        // Re-entered (fail safe)
        I2S_OUT_PULSER_EXIT_CRITICAL();
        return;
    }

    if(i2s_sr.pulser_status == PASSTHROUGH)
        gdma_ll_tx_stop(&GDMA, i2s_sr.dma.channel);

    // Wait for complete DMAs
    while(i2s_sr.pulser_status == WAITING) {
        I2S_OUT_PULSER_EXIT_CRITICAL();
        delay(I2S_OUT_DELAY_DMABUF_MS);
        I2S_OUT_PULSER_ENTER_CRITICAL();
    }

    // Change I2S to STEPPING
    i2s_out_start(STEPPING);

    I2S_OUT_PULSER_EXIT_CRITICAL();
}

void IRAM_ATTR i2s_out_set_pulse_period (uint32_t period)
{
    i2s_sr.pulse_period = period;
}

void IRAM_ATTR i2s_out_set_pulse_callback (i2s_out_pulse_func_t func)
{
    i2s_sr.pulse_func = func;
}

void IRAM_ATTR i2s_out_reset (void)
{
    I2S_OUT_PULSER_ENTER_CRITICAL();

    if (i2s_sr.pulser_status == STEPPING) {
        uint32_t port_data = atomic_load(&i2s_sr.port_data);
        i2s_clear_o_dma_buffers(port_data);
    }

    // You need to set the status before calling i2s_out_start()
    // because the process in i2s_out_start() is different depending on the status.
    i2s_out_start(i2s_sr.pulser_status == WAITING ? PASSTHROUGH : i2s_sr.pulser_status);

    I2S_OUT_PULSER_EXIT_CRITICAL();
}

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

//
// Initialize funtion (external function)
//
bool IRAM_ATTR i2s_out_init2 (i2s_out_init_t init_param)
{
    if(i2s_sr.initialized)
        return false;

    if((i2s_sr.dma.channel = allocate_dma_channel()) == -1)
        return false;

    atomic_store(&i2s_sr.port_data, init_param.init_val);

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
    i2s_out_gpio_attach(init_param.ws_pin, init_param.bck_pin, init_param.data_pin);

    /**
   * Each i2s transfer will take
   *   fpll = PLL_D2_CLK      -- clka_en = 0
   *
   *   fi2s = fpll / N + b/a  -- N + b/a = clkm_div_num
   *   fi2s = 160MHz / 2
   *   fi2s = 80MHz
   *
   *   fbclk = fi2s / M   -- M = tx_bck_div_num
   *   fbclk = 80MHz / 2
   *   fbclk = 40MHz
   *
   *   fwclk = fbclk / 32
   *
   *   for fwclk = 250kHz(16-bit: 4uS pulse time), 125kHz(32-bit: 8uS pulse time)
   *      N = 10, b/a = 0
   *      M = 2
   *   for fwclk = 500kHz(16-bit: 2uS pulse time), 250kHz(32-bit: 4uS pulse time)
   *      N = 5, b/a = 0
   *      M = 2
   *   for fwclk = 1000kHz(16-bit: 1uS pulse time), 500kHz(32-bit: 2uS pulse time)
   *      N = 2, b/a = 2/1 (N + b/a = 2.5)
   *      M = 2
   */

    // Allocate the array of pointers to the buffers
    if((i2s_sr.dma.buffers = (uint32_t **)malloc(sizeof(uint32_t *) * I2S_OUT_DMABUF_COUNT)) == NULL)
        return false;

    // Allocate each buffer that can be used by the DMA controller
    for(int i = 0; i < I2S_OUT_DMABUF_COUNT; i++) {
        if((i2s_sr.dma.buffers[i] = (uint32_t *)heap_caps_calloc(1, I2S_OUT_DMABUF_LEN, MALLOC_CAP_DMA)) == NULL)
            return false;
    }

    // Allocate the array of DMA descriptors
    if((i2s_sr.dma.desc = (dma_descriptor_t **)malloc(sizeof(dma_descriptor_t *) * I2S_OUT_DMABUF_COUNT)) == NULL)
        return false;

    // Allocate each DMA descriptor that will be used by the DMA controller
    for(int i = 0; i < I2S_OUT_DMABUF_COUNT; i++) {
        if((i2s_sr.dma.desc[i] = (dma_descriptor_t *)heap_caps_malloc(sizeof(dma_descriptor_t), MALLOC_CAP_DMA)) == NULL)
            return false;
    }

    // Initialize
    i2s_clear_o_dma_buffers(init_param.init_val);
    i2s_sr.dma.rw_pos  = 0;
    i2s_sr.dma.current = NULL;
#if !I2S_LOCAL_QUEUE
    i2s_sr.dma.queue   = xQueueCreate(I2S_OUT_DMABUF_COUNT, sizeof(uint32_t *));
#endif

    i2s_ll_tx_stop(&I2S0);

//    gdma_ahb_hal_connect_peri(&GDMA, i2s_sr.dma.channel, GDMA_CHANNEL_DIRECTION_TX, GDMA_TRIG_PERIPH_I2S, SOC_GDMA_TRIG_PERIPH_I2S0);

    gdma_ll_enable_clock(&GDMA, 1);
    gdma_ll_tx_reset_channel(&GDMA, i2s_sr.dma.channel);
    gdma_ll_tx_enable_interrupt(&GDMA, i2s_sr.dma.channel, GDMA_LL_TX_EVENT_MASK, false);
    gdma_ll_tx_clear_interrupt_status(&GDMA, i2s_sr.dma.channel, GDMA_LL_TX_EVENT_MASK);
    gdma_ll_tx_connect_to_periph(&GDMA, i2s_sr.dma.channel, GDMA_TRIG_PERIPH_I2S, SOC_GDMA_TRIG_PERIPH_I2S0);
    gdma_ll_tx_set_eof_mode(&GDMA, i2s_sr.dma.channel, 0); // Not needed?

#if I2S_OUT_NUM_BITS == 16
    I2S0.fifo_conf.tx_fifo_mod        = 0;   // 0: 16-bit dual channel data, 3: 32-bit single channel data
    I2S0.fifo_conf.rx_fifo_mod        = 0;   // 0: 16-bit dual channel data, 3: 32-bit single channel data
    I2S0.sample_rate_conf.tx_bits_mod = 16;  // default is 16-bits
    I2S0.sample_rate_conf.rx_bits_mod = 16;  // default is 16-bits

#else
    i2s_ll_tx_enable_pdm(&I2S0, false); // Enables TDM
    i2s_ll_rx_set_active_chan_mask(&I2S0, 1);
    i2s_ll_tx_enable_msb_shift(&I2S0, 1);
    i2s_ll_tx_set_sample_bit(&I2S0, 32, 32); // ?
    i2s_ll_tx_set_ws_width(&I2S0, 1);
#endif

    //
    // i2s_set_clk
    //
    i2s_ll_tx_set_ws_idle_pol(&I2S0, 0);
    i2s_ll_tx_clk_set_src(&I2S0, I2S_CLK_D2CLK); // Set I2S_CLK_D2CLK as default
    i2s_ll_mclk_use_tx_clk(&I2S0);

#if I2S_OUT_NUM_BITS == 16
    // N = 10
    //!     I2S0.clkm_conf.clkm_div_num = 10;  // minimum value of 2, reset value of 4, max 256 (I²S clock divider’s integral value)
#else
    // N = 5
    //!     I2S0.clkm_conf.clkm_div_num = 5;  // minimum value of 2, reset value of 4, max 256 (I²S clock divider’s integral value)
    i2s_ll_mclk_div_t clk_ = {
         .mclk_div = 2,
         .a = 20,
         .b = 17
    };
#endif

    i2s_ll_tx_set_clk(&I2S0, &clk_);
    i2s_ll_tx_enable_clock(&I2S0);
    i2s_ll_tx_reset(&I2S0);
    i2s_ll_tx_reset_fifo(&I2S0);
    I2S0.tx_conf.tx_stop_en = 1;

#if I2S_LOCAL_QUEUE

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = i2s_poll_rt;

//    on_execute_delay = grbl.on_execute_delay;
//    grbl.on_execute_delay = i2s_poll_dly;

#else
    // Create the task that will feed the buffer
    xTaskCreatePinnedToCore(i2sOutTask,
                            "I2SOutTask",
                            4096,
                            NULL,
                            GRBLHAL_TASK_PRIORITY + 1,
                            NULL,
                            GRBLHAL_TASK_CORE  // must run the task on same core
    );

#endif

    esp_err_t ret;

    // Allocate and enable the I2S DMA interrupt

    ret = esp_intr_alloc(gdma_periph_signals.groups[0].pairs[i2s_sr.dma.channel].tx_irq_id, /*ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_INTRDISABLED*/0, i2s_out_intr_handler, NULL, &i2s_sr.dma.intr_handle);
/*    esp_intr_alloc_intrstatus(gdma_periph_signals.groups[0].pairs[i2s_sr.dma.channel].tx_irq_id, ESP_INTR_FLAG_INTRDISABLED,
                                        (uint32_t)gdma_ll_tx_get_interrupt_status_reg(&GDMA, i2s_sr.dma.channel), GDMA_LL_TX_EVENT_MASK,
                                        i2s_out_intr_handler, NULL, &i2s_sr.dma.intr_handle);*/
//    esp_intr_enable(i2s_sr.dma.intr_handle);

    // Default pulse callback period (usec)
    i2s_sr.pulse_period = init_param.pulse_period;
    i2s_sr.pulse_func   = init_param.pulse_func;

    // Remember GPIO pin numbers
    i2s_sr.ws_pin      = init_param.ws_pin;
    i2s_sr.bck_pin     = init_param.bck_pin;
    i2s_sr.data_pin    = init_param.data_pin;
    i2s_sr.initialized = true;

    // Transmit recovery data to 74HC595
    i2s_out_gpio_shiftout((pd = init_param.init_val));

    // Start the I2S peripheral
    debug_writeln("hi");
    return i2s_out_start(PASSTHROUGH);
}

/*
  Initialize I2S out by default parameters.

  return false ... already initialized
*/
bool IRAM_ATTR i2s_out_init (void)
{
    i2s_out_init_t default_param = {
        .ws_pin       = I2S_OUT_WS,
        .bck_pin      = I2S_OUT_BCK,
        .data_pin     = I2S_OUT_DATA,
        .pulse_func   = NULL,
        .pulse_period = I2S_OUT_USEC_PER_PULSE,
        .init_val     = I2S_OUT_INIT_VAL
    };

    return i2s_out_init2(default_param);
}

#endif
