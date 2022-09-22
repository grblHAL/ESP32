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

//static const DRAM_ATTR uint16_t RX_BUFFER_SIZE_MASK = RX_BUFFER_SIZE - 1;

static uart_t *uart1 = NULL;
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
static uart_config_t uart1Config = {
    .baud_rate=BAUD_RATE,
    .data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE,
    .stop_bits=UART_STOP_BITS_2,
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh=0,
};
static QueueHandle_t _uart1_queue;

#if SERIAL2_ENABLE
static uart_t *uart2 = NULL;
static stream_rx_buffer_t rxbuffer2 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;
static uart_config_t uart2Config = {
    .baud_rate=BAUD_RATE,
    .data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE,
    .stop_bits=UART_STOP_BITS_2,
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh=0,
};
static QueueHandle_t _uart2_queue;
#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
xSemaphoreHandle _uart2_modbus_lock = xSemaphoreCreateMutex();
#endif
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

IRAM_ATTR static void _uart1_handle_isr(void *arg){
    uart_event_t event;
    char data[RX_BUFFER_SIZE]={0};
    int i=0;
    for(;;){
        if(xQueueReceive(_uart1_queue, (void *)&event, (portTickType)portMAX_DELAY)){
            uart_read_bytes(uart1->num, data, event.size, portMAX_DELAY);
            for(i=0;i<event.size;i++){
                char c = data[i];
                if(!enqueue_realtime_command(c)) {
                    uint32_t bptr = BUFNEXT(rxbuffer.head, rxbuffer);
                    if(bptr == rxbuffer.tail)                   // If buffer full
                        rxbuffer.overflow = 1;                  // flag overflow,
                    else {
                        rxbuffer.data[rxbuffer.head] = (char)c; // else add data to buffer
                        rxbuffer.head = bptr;                   // and update pointer
                    }
                }
            }
        }
    }
}

static void uartEnableInterrupt (uart_t *uart, bool enable_rx)
{
    UART_MUTEX_LOCK(uart);
    uart_intr_config_t intConfig = {
        .rx_timeout_thresh = 50,
        .rxfifo_full_thresh = 112,
        .intr_enable_mask = UART_INTR_RXFIFO_FULL|UART_INTR_FRAM_ERR|UART_INTR_RXFIFO_TOUT,
    };
    intConfig.intr_enable_mask *= enable_rx;
    uart_intr_config(uart->num, &intConfig);
    UART_MUTEX_UNLOCK(uart);
}
/*
static void uartDisableInterrupt (uart_t *uart)
{
    UART_MUTEX_LOCK();
    uart_disable_rx_intr(uart->num);
    uart_disable_intr_mask(uart->num, UART_INTR_RXFIFO_FULL|
                                      UART_INTR_FRAM_ERR   |
                                      UART_INTR_RXFIFO_TOUT);
    UART_MUTEX_UNLOCK();
}
*/
static void uartSetBaudRate (uart_t *uart, uint32_t baud_rate)
{
    if(uart == NULL)
        return;

    UART_MUTEX_LOCK(uart);
    uart_set_baudrate(uart->num, baud_rate);
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
    switch(uart->num){
        case 0:
            uart_param_config(uart->num, &uart1Config);
            uart_set_pin(uart->num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            break;
        case 1:
#if SERIAL2_ENABLE
            uart_param_config(uart->num, &uart2Config);
  #ifdef UART2_TX_PIN
            uart_set_pin(uart->num, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  #else
            uart_set_pin(uart->num, UART_PIN_NO_CHANGE, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  #endif
#endif
            break;
        default:
            break;
    }
    return;

    UART_MUTEX_UNLOCK(uart);
}

inline IRAM_ATTR static void txFlush (uart_t *uart)
{
    uart_tx_flush(uart->num);
}

inline IRAM_ATTR static void rxFlush (uart_t *uart)
{
    uart_flush(uart->num);
}

IRAM_ATTR static void flush (uart_t *uart)
{
    txFlush(uart);
    rxFlush(uart);
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

static bool serialSetBaudRate (uint32_t baudrate)
{
    uartSetBaudRate(uart1, baudrate);
    return true; //FIXME: return the result of the baudrate set
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
    uart_write_bytes(uart1->num, &c, 1);//already waits for space on txRingBuffer
    return true;
}

static void serialWriteS (const char *data)
{
    int n = strlen(data);
    for(int i=0;i<n;i++)
        serialPutC(data[i]);
}

IRAM_ATTR static void serialFlush (void)
{
    UART_MUTEX_LOCK(uart1);

    flush(uart1);
    rxbuffer.tail = rxbuffer.head;

    UART_MUTEX_UNLOCK(uart1);
}

IRAM_ATTR static void serialTxFlush (void)
{
    UART_MUTEX_LOCK(uart1);

    txFlush(uart1);

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
        uart_disable_intr_mask(uart1->num, UART_INTR_RXFIFO_FULL|UART_INTR_FRAM_ERR|UART_INTR_RXFIFO_TOUT);
    } else {
        // Clear and enable interrupts
        uart_clear_intr_status(uart1->num, UART_INTR_RXFIFO_FULL|UART_INTR_FRAM_ERR|UART_INTR_RXFIFO_TOUT);
        flush(uart1);
        rxbuffer.tail = rxbuffer.head;
        uart_enable_intr_mask(uart1->num, UART_INTR_RXFIFO_FULL|UART_INTR_FRAM_ERR|UART_INTR_RXFIFO_TOUT);
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
        //.write_n =  serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRXFree,
        .get_rx_buffer_count = serialAvailable,
        //.get_tx_buffer_count = serialTxCount,
        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialFlush,
        .cancel_read_buffer = serialCancel,
        .suspend_read = serialSuspendInput,
        .set_baud_rate = serialSetBaudRate,
        .disable_rx = serialDisable,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    uart1 = &_uart_bus_array[0]; // use UART 0

    uartConfig(uart1, baud_rate);
    uart_driver_install(uart1->num, RX_BUFFER_SIZE, TX_BUFFER_SIZE, RX_BUFFER_SIZE, &_uart1_queue, 0);

    serialFlush();
    xTaskCreate(_uart1_handle_isr, "uart1HandleIsr", 2*RX_BUFFER_SIZE, NULL, 10, NULL);
    uartEnableInterrupt(uart1, true);

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

IRAM_ATTR static void _uart2_handle_isr(void *arg){
    uart_event_t event;
    char data[RX_BUFFER_SIZE]={0};
    int i=0;
    for(;;){
        if(xQueueReceive(_uart2_queue, (void *)&event, (portTickType)portMAX_DELAY)){
#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
            if(xSemaphoreGive(_uart2_modbus_lock,0)==pdTRUE && uart_wait_tx_done(uart2->num, 0)==ESP_OK){
                gpio_set_level(MODBUS_DIRECTION_PIN, false);
            }
#endif
            uart_read_bytes(uart2->num, data, event.size, portMAX_DELAY);
            for(i=0;i<event.size;i++){
                char c = data[i];
                if(!enqueue_realtime_command2(c)) {
                    uint32_t bptr = BUFNEXT(rxbuffer2.head, rxbuffer2);
                    if(bptr == rxbuffer2.tail)                   // If buffer full
                        rxbuffer2.overflow = 1;                  // flag overflow,
                    else {
                        rxbuffer2.data[rxbuffer2.head] = (char)c; // else add data to buffer
                        rxbuffer2.head = bptr;                   // and update pointer
                    }
                }
            }
        }
    }
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
    uart_write_bytes(uart2->num, &c, 1);//already waits for space on txRingBuffer
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

    //TODO:active mutex that will set MODBUS_DIRECTION_PIN to low on the isrHandler
#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
    xSemaphoreTake(_uart2_modbus_lock,0);
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
        uart_disable_intr_mask(uart2->num, UART_INTR_RXFIFO_FULL|UART_INTR_FRAM_ERR|UART_INTR_RXFIFO_TOUT);
    } else {
        // Clear and enable interrupts
        uart_clear_intr_status(uart2->num, UART_INTR_RXFIFO_FULL|UART_INTR_FRAM_ERR|UART_INTR_RXFIFO_TOUT);
        flush(uart2);
        rxbuffer2.tail = rxbuffer2.head;
        uart_enable_intr_mask(uart2->num, UART_INTR_RXFIFO_FULL|UART_INTR_FRAM_ERR|UART_INTR_RXFIFO_TOUT);
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
    uart_driver_install(uart2->num, RX_BUFFER_SIZE, TX_BUFFER_SIZE, RX_BUFFER_SIZE, &_uart2_queue, 0);

    serial2Flush();
    xTaskCreate(_uart2_handle_isr, "uart2HandleIsr", 2*RX_BUFFER_SIZE, NULL, 10, NULL);
#ifdef UART2_TX_PIN
    uartEnableInterrupt(uart2, true);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .pin = UART2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&tx);
#else
    uartEnableInterrupt(uart2, false);
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
