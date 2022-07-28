// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
// Copyright 2018-2021 Terje Io : Modifications for grbl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp32/rom/ets_sys.h"
#include "esp_attr.h"
#include "esp32/rom/uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "esp_intr_alloc.h"

#include "esp32-hal-uart.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#define TWO_STOP_BITS_CONF 0x3
#define ONE_STOP_BITS_CONF 0x1
#define CONFIG_DISABLE_HAL_LOCKS 1

#define UART_REG_BASE(u)    ((u==0)?DR_REG_UART_BASE:(      (u==1)?DR_REG_UART1_BASE:(    (u==2)?DR_REG_UART2_BASE:0)))
#define UART_RXD_IDX(u)     ((u==0)?U0RXD_IN_IDX:(          (u==1)?U1RXD_IN_IDX:(         (u==2)?U2RXD_IN_IDX:0)))
#define UART_TXD_IDX(u)     ((u==0)?U0TXD_OUT_IDX:(         (u==1)?U1TXD_OUT_IDX:(        (u==2)?U2TXD_OUT_IDX:0)))
#define UART_INTR_SOURCE(u) ((u==0)?ETS_UART0_INTR_SOURCE:( (u==1)?ETS_UART1_INTR_SOURCE:((u==2)?ETS_UART2_INTR_SOURCE:0)))

typedef void (*uart_isr_ptr)(void *arg);

typedef struct {
    uart_dev_t *dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    intr_handle_t intr_handle;
} uart_t;

static int16_t serialRead (void);

#if CONFIG_DISABLE_HAL_LOCKS
#define UART_MUTEX_LOCK(u)
#define UART_MUTEX_UNLOCK(u)

static uart_t _uart_bus_array[3] = {
    {(volatile uart_dev_t *)(DR_REG_UART_BASE), 0, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART1_BASE), 1, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART2_BASE), 2, NULL}
};
#else
#define UART_MUTEX_LOCK(u)    do {} while (xSemaphoreTake(u->lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK(u)  xSemaphoreGive(u->lock)

static uart_t _uart_bus_array[3] = {
    {(volatile uart_dev_t *)(DR_REG_UART_BASE), NULL, 0, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART1_BASE), NULL, 1, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART2_BASE), NULL, 2, NULL}
};
#endif

static const DRAM_ATTR uint16_t RX_BUFFER_SIZE_MASK = RX_BUFFER_SIZE - 1;

static uart_t *uart1 = NULL;
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

#if SERIAL2_ENABLE
static uart_t *uart2 = NULL;
static stream_rx_buffer_t rxbuffer2 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;
#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .claim = serialInit
    },
#ifdef SERIAL2_ENABLE
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
#ifdef UART2_TX_PIN
      .flags.modbus_ready = On,
#else
      .flags.rx_only = On,
#endif
      .claim = serial2Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

IRAM_ATTR static void _uart1_isr (void *arg)
{
    uint8_t c;

    uart1->dev->int_clr.rxfifo_full = 1;
    uart1->dev->int_clr.frm_err = 1;
    uart1->dev->int_clr.rxfifo_tout = 1;

    while(uart1->dev->status.rxfifo_cnt || (uart1->dev->mem_rx_status.wr_addr != uart1->dev->mem_rx_status.rd_addr)) {

        c = uart1->dev->fifo.rw_byte;

        if(!enqueue_realtime_command(c)) {

            uint32_t bptr = (rxbuffer.head + 1) & RX_BUFFER_SIZE_MASK;  // Get next head pointer

            if(bptr == rxbuffer.tail)                   // If buffer full
                rxbuffer.overflow = 1;                  // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = (char)c; // else add data to buffer
                rxbuffer.head = bptr;                   // and update pointer
            }
        }
    }
}

static void uartEnableInterrupt (uart_t *uart, uart_isr_ptr isr, bool enable_rx)
{
    UART_MUTEX_LOCK(uart);

    esp_intr_alloc(UART_INTR_SOURCE(uart->num), (int)ESP_INTR_FLAG_IRAM, isr, NULL, &uart->intr_handle);

    uart->dev->conf1.rxfifo_full_thrhd = 112;
    uart->dev->conf1.rx_tout_thrhd = 50;
    uart->dev->conf1.rx_tout_en = 1;
    uart->dev->int_ena.rxfifo_full = enable_rx;
    uart->dev->int_ena.frm_err = enable_rx;
    uart->dev->int_ena.rxfifo_tout = enable_rx;
    uart->dev->int_clr.val = 0xffffffff;

    UART_MUTEX_UNLOCK(uart);
}
/*
static void uartDisableInterrupt (uart_t *uart)
{
    UART_MUTEX_LOCK();
    rx_uart->dev->conf1.val = 0;
    rx_uart->dev->int_ena.val = 0;
    rx_uart->dev->int_clr.val = 0xffffffff;

    esp_intr_free(rx_uart->intr_handle);
    rx_uart->intr_handle = NULL;

    UART_MUTEX_UNLOCK();
}
*/
static void uartSetBaudRate (uart_t *uart, uint32_t baud_rate)
{
    if(uart == NULL)
        return;

    UART_MUTEX_LOCK(uart);
    uint32_t clk_div = ((UART_CLK_FREQ << 4) / baud_rate);
    uart->dev->clk_div.div_int = clk_div >> 4 ;
    uart->dev->clk_div.div_frag = clk_div & 0xf;
    UART_MUTEX_UNLOCK(uart);
}

static void uartConfig (uart_t *uart, uint32_t baud_rate)
{
#if !CONFIG_DISABLE_HAL_LOCKS
    if(uart->lock == NULL) {
        uart->lock = xSemaphoreCreateMutex();
        if(uart->lock == NULL)
            return;
    }
#endif

    switch(uart->num) {

        case 0:
            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART_CLK_EN);
            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART_RST);
            break;

        case 1:
            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART1_CLK_EN);
            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART1_RST);
            break;

        case 2:
            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART2_CLK_EN);
            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART2_RST);
            break;
    }

    uartSetBaudRate(uart, baud_rate);

    UART_MUTEX_LOCK(uart);
    uart->dev->conf0.val = SERIAL_8N2;

    if(uart->dev->conf0.stop_bit_num == TWO_STOP_BITS_CONF) {
        uart->dev->conf0.stop_bit_num = ONE_STOP_BITS_CONF;
        uart->dev->rs485_conf.dl1_en = 1;
    }

    // Note: UART0 pin mappings are set at boot, no need to set here unless override is required

#if SERIAL2_ENABLE
    if(uart->num == 1)
  #ifdef UART2_TX_PIN
        uart_set_pin(uart->num, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  #else
        uart_set_pin(uart->num, UART_PIN_NO_CHANGE, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  #endif
#endif

    UART_MUTEX_UNLOCK(uart);
}

IRAM_ATTR static void flush (uart_t *uart)
{
    while(uart->dev->status.txfifo_cnt);

    //Due to hardware issue, we can not use fifo_rst to reset uart fifo.
    //See description about UART_TXFIFO_RST and UART_RXFIFO_RST in <<esp32_technical_reference_manual>> v2.6 or later.

    // we read the data out and make `fifo_len == 0 && rd_addr == wr_addr`.
    while(uart->dev->status.rxfifo_cnt || (uart->dev->mem_rx_status.wr_addr != uart->dev->mem_rx_status.rd_addr))
        READ_PERI_REG(UART_FIFO_REG(uart->num));

    uart_ll_rxfifo_rst(uart->dev);
}

static uint16_t serialAvailable (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRXFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}
/*
static uint32_t serialAvailableForWrite (void)
{
    return uart1 ? 0x7f - uart1->dev->status.txfifo_cnt : 0;
}
*/
static int16_t serialRead (void)
{
    int16_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head) {;
        return -1; // no data available else EOF
    }
    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    return data;
}

static bool serialPutC (const char c)
{
    while(uart1->dev->status.txfifo_cnt == 0x7F) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    uart1->dev->fifo.rw_byte = c;

    return true;
}

static void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

IRAM_ATTR static void serialFlush (void)
{
    UART_MUTEX_LOCK(uart1);

    flush(uart1);
    rxbuffer.tail = rxbuffer.head;

    UART_MUTEX_UNLOCK(uart1);
}

IRAM_ATTR static void serialCancel (void)
{
//    UART_MUTEX_LOCK(uart1);
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
//    UART_MUTEX_UNLOCK(uart1);
}

IRAM_ATTR static bool serialSuspendInput (bool suspend)
{
    bool ok;
    UART_MUTEX_LOCK(uart1);
    ok = stream_rx_suspend(&rxbuffer, suspend);
    UART_MUTEX_UNLOCK(uart1);

    return ok;
}

IRAM_ATTR static bool serialDisable (bool disable)
{
    UART_MUTEX_LOCK(uart1);

    if(disable) {
        // Disable interrupts
        uart1->dev->int_ena.rxfifo_full = 0;
        uart1->dev->int_ena.frm_err = 0;
        uart1->dev->int_ena.rxfifo_tout = 0;
    } else {
        // Clear and enable interrupts
        uart1->dev->int_ena.rxfifo_full = 0;
        uart1->dev->int_clr.rxfifo_full = 1;
        uart1->dev->int_clr.frm_err = 1;
        uart1->dev->int_clr.rxfifo_tout = 1;
        flush(uart1);
        rxbuffer.tail = rxbuffer.head;
        uart1->dev->int_ena.rxfifo_full = 1;
        uart1->dev->int_ena.frm_err = 1;
        uart1->dev->int_ena.rxfifo_tout = 1;
    }

    UART_MUTEX_UNLOCK(uart1);

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = true,
        .read = serialRead,
        .write = serialWriteS,
//        .write_n =  serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRXFree,
        .get_rx_buffer_count = serialAvailable,
//        .get_tx_buffer_count = serialTxCount,
//        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialFlush,
        .cancel_read_buffer = serialCancel,
        .suspend_read = serialSuspendInput,
    //    .set_baud_rate = serialSetBaudRate
        .disable_rx = serialDisable,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    uart1 = &_uart_bus_array[0]; // use UART 0

    uartConfig(uart1, baud_rate);

    serialFlush();
    uartEnableInterrupt(uart1, _uart1_isr, true);
    
    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .pin = 35,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .pin = 34,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

#if SERIAL2_ENABLE

static void IRAM_ATTR _uart2_isr (void *arg)
{
    uint8_t c;

    uart2->dev->int_clr.rxfifo_full = 1;
    uart2->dev->int_clr.frm_err = 1;
    uart2->dev->int_clr.rxfifo_tout = 1;

#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
    if(uart2->dev->int_st.tx_done) {
    //    uart2->dev->int_clr.tx_done = 1;
        uart2->dev->int_ena.tx_done = 0;
        gpio_set_level(MODBUS_DIRECTION_PIN, false);
    }
#endif

    while(uart2->dev->status.rxfifo_cnt || (uart2->dev->mem_rx_status.wr_addr != uart2->dev->mem_rx_status.rd_addr)) {

        c = uart2->dev->fifo.rw_byte;

        if(!enqueue_realtime_command2(c)) {

            uint32_t bptr = (rxbuffer2.head + 1) & RX_BUFFER_SIZE_MASK;  // Get next head pointer

            if(bptr == rxbuffer2.tail)                    // If buffer full
                rxbuffer2.overflow = 1;                   // flag overflow,
            else {
                rxbuffer2.data[rxbuffer2.head] = (char)c; // else add data to buffer
                rxbuffer2.head = bptr;                    // and update pointer
            }
        }
    }

/*
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    */
}

uint16_t static serial2Available (void)
{
    uint16_t head = rxbuffer2.head, tail = rxbuffer2.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t static serial2txCount (void)
{
    return (uint16_t)uart2->dev->status.txfifo_cnt + (uart2->dev->status.st_utx_out ? 1 : 0);
}

uint16_t static serial2RXFree (void)
{
    uint16_t head = rxbuffer2.head, tail = rxbuffer2.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

bool static serial2PutC (const char c)
{
    UART_MUTEX_LOCK(uart2);

    while(uart2->dev->status.txfifo_cnt == 0x7F);

    uart2->dev->fifo.rw_byte = c;

    UART_MUTEX_UNLOCK(uart2);

    return true;
}

void static serial2WriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

//
// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
void static serial2Write (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
    gpio_set_level(MODBUS_DIRECTION_PIN, true);
#endif

    while(length--)
        serial2PutC(*ptr++);

#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
    uart2->dev->int_clr.tx_done = 1;
    uart2->dev->int_ena.tx_done = 1;
#endif
}

int16_t static serial2Read (void)
{
    UART_MUTEX_LOCK(uart2);

    int16_t data;
    uint16_t bptr = rxbuffer2.tail;

    if(bptr == rxbuffer2.head) {
        UART_MUTEX_UNLOCK(uart2);
        return -1; // no data available else EOF
    }

    data = rxbuffer2.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer2.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    UART_MUTEX_UNLOCK(uart2);

    return data;
}

IRAM_ATTR static void serial2Flush (void)
{
    UART_MUTEX_LOCK(uart2);

    flush(uart2);
    rxbuffer2.tail = rxbuffer2.head;

    UART_MUTEX_UNLOCK(uart2);
}

IRAM_ATTR static void serial2Cancel (void)
{
//    UART_MUTEX_LOCK(uart2);
    rxbuffer2.data[rxbuffer2.head] = ASCII_CAN;
    rxbuffer2.tail = rxbuffer2.head;
    rxbuffer2.head = (rxbuffer2.tail + 1) & (RX_BUFFER_SIZE - 1);
//    UART_MUTEX_UNLOCK(uart2);
}

static bool serial2SuspendInput (bool suspend)
{
    bool ok;

    UART_MUTEX_LOCK(uart2);

    ok = stream_rx_suspend(&rxbuffer2, suspend);

    UART_MUTEX_UNLOCK(uart2);

    return ok;
}

IRAM_ATTR static bool serial2Disable (bool disable)
{
    UART_MUTEX_LOCK(uart2);

    if(disable) {
        // Disable interrupts
        uart2->dev->int_ena.rxfifo_full = 0;
        uart2->dev->int_ena.frm_err = 0;
        uart2->dev->int_ena.rxfifo_tout = 0;
    } else {
        // Clear and enable interrupts
        uart2->dev->int_ena.rxfifo_full = 0;
        uart2->dev->int_clr.rxfifo_full = 1;
        uart2->dev->int_clr.frm_err = 1;
        uart2->dev->int_clr.rxfifo_tout = 1;
        flush(uart2);
        rxbuffer2.tail = rxbuffer2.head;
        uart2->dev->int_ena.rxfifo_full = 1;
        uart2->dev->int_ena.frm_err = 1;
        uart2->dev->int_ena.rxfifo_tout = 1;
    }

    UART_MUTEX_UNLOCK(uart2);

    return true;
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
    static bool init_ok = false;

    if(!init_ok) {
        serial2Init(baud_rate);
        init_ok = true;
    }

    uartSetBaudRate(uart2, baud_rate);

    return true;
}

static bool serial2EnqueueRtCommand (char c)
{
    return enqueue_realtime_command2(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = true,
        .read = serial2Read,
        .write = serial2WriteS,
        .write_n =  serial2Write,
        .write_char = serial2PutC,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RXFree,
        .get_rx_buffer_count = serial2Available,
        .get_tx_buffer_count = serial2txCount,
        .reset_write_buffer = serial2Flush,
        .reset_read_buffer = serial2Flush,
        .cancel_read_buffer = serial2Cancel,
        .suspend_read = serial2SuspendInput,
        .set_baud_rate = serial2SetBaudRate,
        .disable_rx = serial2Disable,
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    if(serial[1].flags.claimed)
        return NULL;

    serial[1].flags.claimed = On;

    uart2 = &_uart_bus_array[1]; // use UART 1

    uartConfig(uart2, baud_rate);

    serial2Flush();
#ifdef UART2_TX_PIN
    uartEnableInterrupt(uart2, _uart2_isr, true);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .pin = UART2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&tx);
#else
    uartEnableInterrupt(uart2, _uart2_isr, false);
#endif

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .pin = UART2_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&rx);

    return &stream;
}

#endif
