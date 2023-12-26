/*

  uart_serial.c - driver code for ESP32

  Part of grblHAL

  Copyright (c) 2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#if !GRBL_ESP32S3
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/uart.h"
#endif
#include "esp_attr.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "esp_intr_alloc.h"

#include "driver.h"
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
#if GRBL_ESP32S3
   uart_dev_t *dev;
#else
   volatile uart_dev_t *dev;
#endif
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    intr_handle_t intr_handle;
    uint32_t tx_len;
} uart_t;

static int16_t serialRead (void);

#if CONFIG_DISABLE_HAL_LOCKS

#define UART_MUTEX_LOCK(u)
#define UART_MUTEX_UNLOCK(u)

static const uart_t _uart_bus_array[3] = {
    { (uart_dev_t *)(DR_REG_UART_BASE), 0, NULL, SOC_UART_FIFO_LEN },
    { (uart_dev_t *)(DR_REG_UART1_BASE), 1, NULL, SOC_UART_FIFO_LEN },
    { (uart_dev_t *)(DR_REG_UART2_BASE), 2, NULL, SOC_UART_FIFO_LEN }
};

#else

#define UART_MUTEX_LOCK(u)    do {} while (xSemaphoreTake((u)->lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK(u)  xSemaphoreGive((u)->lock)

static const uart_t _uart_bus_array[3] = {
    { (uart_dev_t *)(DR_REG_UART_BASE), NULL, 0, NULL, SOC_UART_FIFO_LEN },
    { (uart_dev_t *)(DR_REG_UART1_BASE), NULL, 1, NULL, SOC_UART_FIFO_LEN },
    { (uart_dev_t *)(DR_REG_UART2_BASE), NULL, 2, NULL, SOC_UART_FIFO_LEN }
};

#endif

static const DRAM_ATTR uint16_t RX_BUFFER_SIZE_MASK = RX_BUFFER_SIZE - 1;
static const DRAM_ATTR uint32_t rx_int_flags = UART_INTR_RXFIFO_FULL|UART_INTR_RXFIFO_OVF|UART_INTR_RXFIFO_TOUT|UART_INTR_FRAM_ERR;

static uart_t uart1;
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
static const io_stream_t *serialInit (uint32_t baud_rate);

#if SERIAL2_ENABLE
static uart_t uart2;
static stream_rx_buffer_t rxbuffer2 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;
static const io_stream_t *serial2Init (uint32_t baud_rate);
#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serialInit
    },
#if SERIAL2_ENABLE
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
#endif // SERIAL2_ENABLE
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    static const periph_pin_t tx0 = {
        .function = Output_TX,
        .group = PinGroup_UART,
#if GRBL_ESP32S3
        .pin = 43,
#else
        .pin = 35,
#endif
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx0 = {
        .function = Input_RX,
        .group = PinGroup_UART,
#if GRBL_ESP32S3
        .pin = 44,
#else
        .pin = 34,
#endif
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

    hal.periph_port.register_pin(&rx0);
    hal.periph_port.register_pin(&tx0);

#if SERIAL2_ENABLE

  #ifdef UART2_TX_PIN
    static const periph_pin_t tx1 = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .pin = UART2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&tx1);
  #endif

    static const periph_pin_t rx1 = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .pin = UART2_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&rx1);

#endif // SERIAL2_ENABLE

    stream_register_streams(&streams);
}

static void uartSetBaudRate (uart_t *uart, uint32_t baud_rate)
{
    if(uart == NULL)
        return;

    UART_MUTEX_LOCK(uart);
    uart_ll_set_baudrate(uart->dev, baud_rate);
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

//    uart->tx_len = 128;
    uart_ll_set_mode(uart->dev, UART_MODE_UART);

#if GRBL_ESP32S3
#else
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
#endif

    uartSetBaudRate(uart, baud_rate);

    UART_MUTEX_LOCK(uart);

    uart_ll_set_data_bit_num(uart->dev, UART_DATA_8_BITS);
    uart_ll_set_stop_bits(uart->dev, UART_STOP_BITS_2);
    uart_ll_set_parity(uart->dev, UART_PARITY_DISABLE);

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

static void uartEnableInterrupt (uart_t *uart, uart_isr_ptr isr, bool enable_rx)
{
    UART_MUTEX_LOCK(uart);

    if(!uart->intr_handle)
        esp_intr_alloc(UART_INTR_SOURCE(uart->num), (int)ESP_INTR_FLAG_IRAM, isr, NULL, &uart->intr_handle);

    uart_ll_set_rxfifo_full_thr(uart->dev, 112);
    uart_ll_set_rx_tout(uart->dev, enable_rx ? 50 : 0);
    uart_ll_clr_intsts_mask(uart->dev, rx_int_flags);
    if(enable_rx)
        uart_ll_ena_intr_mask(uart->dev, rx_int_flags);
    else
        uart_ll_disable_intr_mask(uart->dev, rx_int_flags);

    UART_MUTEX_UNLOCK(uart);
}

IRAM_ATTR static void _uart_flush (uart_t *uart, bool tx_only)
{
    uart_ll_txfifo_rst(uart->dev);
    while(!uart_ll_is_tx_idle(uart->dev));

    if(!tx_only)
        uart_ll_rxfifo_rst(uart->dev);
}

FORCE_INLINE_ATTR uint8_t _uart_ll_read_rxfifo (uart_dev_t *hw)
{
    uint8_t c;
    //Get the UART APB fifo addr. Read fifo, we use APB address
    uint32_t fifo_addr = (hw == &UART0) ? UART_FIFO_REG(0) : (hw == &UART1) ? UART_FIFO_REG(1) : UART_FIFO_REG(2);

    c = READ_PERI_REG(fifo_addr);
#ifdef CONFIG_COMPILER_OPTIMIZATION_PERF
    __asm__ __volatile__("nop");
#endif

    return c;
}

FORCE_INLINE_ATTR void _uart_ll_write_txfifo (uart_dev_t *hw, const uint8_t c)
{
    //Get the UART AHB fifo addr, Write fifo, we use AHB address
    uint32_t fifo_addr = (hw == &UART0) ? UART_FIFO_AHB_REG(0) : (hw == &UART1) ? UART_FIFO_AHB_REG(1) : UART_FIFO_AHB_REG(2);

    WRITE_PERI_REG(fifo_addr, c);
}

FORCE_INLINE_ATTR uint32_t _uart_ll_get_txfifo_count (uart_dev_t *hw)
{
    return HAL_FORCE_READ_U32_REG_FIELD(hw->status, txfifo_cnt);
}

// UART0

IRAM_ATTR static void _uart1_isr (void *arg)
{
    uint32_t c, cnt = uart_ll_get_rxfifo_len(uart1.dev), iflags = uart_ll_get_intsts_mask(uart1.dev);

    uart_ll_clr_intsts_mask(uart1.dev, iflags);

    if(iflags & UART_INTR_RXFIFO_OVF)
        rxbuffer.overflow = On;

    while(cnt) {

        cnt--;
        c = _uart_ll_read_rxfifo(uart1.dev);

        if(!enqueue_realtime_command(c)) {

            uint32_t bptr = (rxbuffer.head + 1) & RX_BUFFER_SIZE_MASK;  // Get next head pointer

            if(bptr == rxbuffer.tail)                   // If buffer full
                rxbuffer.overflow = On;                 // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = (char)c; // else add data to buffer
                rxbuffer.head = bptr;                   // and update pointer
            }
        }
    }
}

static uint16_t serialAvailable (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t static serialTxCount (void)
{
    return uart_ll_is_tx_idle(uart1.dev) ? 0 : (uint16_t)_uart_ll_get_txfifo_count(uart1.dev) + 1;
}

static uint16_t serialRXFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static int16_t serialRead (void)
{
    int16_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    return data;
}

static bool serialPutC (const char c)
{
    while(_uart_ll_get_txfifo_count(uart1.dev) == uart1.tx_len) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    _uart_ll_write_txfifo(uart1.dev, c);

    return true;
}

static void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
       serialPutC(c);
}

//
// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
void static serialWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

IRAM_ATTR static void serialFlush (void)
{
    UART_MUTEX_LOCK(&uart1);

    _uart_flush(&uart1, false);

    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = Off;

    UART_MUTEX_UNLOCK(&uart1);
}

IRAM_ATTR static void serialTxFlush (void)
{
    UART_MUTEX_LOCK(&uart1);

    _uart_flush(&uart1, true);

    UART_MUTEX_UNLOCK(&uart1);
}

IRAM_ATTR static void serialCancel (void)
{
    UART_MUTEX_LOCK(&uart1);

    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);

    UART_MUTEX_UNLOCK(&uart1);
}

IRAM_ATTR static bool serialSuspendInput (bool suspend)
{
    UART_MUTEX_LOCK(&uart1);

    bool ok = stream_rx_suspend(&rxbuffer, suspend);

    UART_MUTEX_UNLOCK(&uart1);

    return ok;
}

IRAM_ATTR static bool serialDisable (bool disable)
{
    UART_MUTEX_LOCK(&uart1);

    uart_ll_disable_intr_mask(uart1.dev, rx_int_flags);

    if(!disable) {
        // Clear and enable interrupts
        _uart_flush(&uart1, false);
        rxbuffer.tail = rxbuffer.head;
        uart_ll_clr_intsts_mask(uart1.dev, rx_int_flags);
        uart_ll_ena_intr_mask(uart1.dev, rx_int_flags);
    }

    UART_MUTEX_UNLOCK(&uart1);

    return true;
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
    uartSetBaudRate(&uart1, baud_rate);

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

static const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = true,
        .read = serialRead,
        .write = serialWriteS,
        .write_n =  serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRXFree,
        .get_rx_buffer_count = serialAvailable,
        .get_tx_buffer_count = serialTxCount,
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

    memcpy(&uart1, &_uart_bus_array[0], sizeof(uart_t)); // use UART 0

    uartConfig(&uart1, baud_rate);

    serialFlush();
    uartEnableInterrupt(&uart1, _uart1_isr, true);

    return &stream;
}

#if SERIAL2_ENABLE

static void IRAM_ATTR _uart2_isr (void *arg)
{
    uint32_t c, cnt = uart_ll_get_rxfifo_len(uart2.dev), iflags = uart_ll_get_intsts_mask(uart2.dev);

    uart_ll_clr_intsts_mask(uart2.dev, iflags);

    if(iflags & UART_INTR_RXFIFO_OVF)
        rxbuffer2.overflow = On;

    while(cnt) {

        cnt--;
        c = _uart_ll_read_rxfifo(uart2.dev);

        if(!enqueue_realtime_command2(c)) {

            uint32_t bptr = (rxbuffer2.head + 1) & RX_BUFFER_SIZE_MASK;  // Get next head pointer

            if(bptr == rxbuffer2.tail)                    // If buffer full
                rxbuffer2.overflow = On;                  // flag overflow,
            else {
                rxbuffer2.data[rxbuffer2.head] = (char)c; // else add data to buffer
                rxbuffer2.head = bptr;                    // and update pointer
            }
        }
    }
}

uint16_t static serial2Available (void)
{
    uint16_t head = rxbuffer2.head, tail = rxbuffer2.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t static serial2TxCount (void)
{
    return uart_ll_is_tx_idle(uart2.dev) ? 0 : (uint16_t)_uart_ll_get_txfifo_count(uart2.dev) + 1;
}

uint16_t static serial2RXFree (void)
{
    uint16_t head = rxbuffer2.head, tail = rxbuffer2.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

bool static serial2PutC (const char c)
{
    UART_MUTEX_LOCK(&uart2);

    while(_uart_ll_get_txfifo_count(uart2.dev) == uart2.tx_len) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    _uart_ll_write_txfifo(uart2.dev, c);

    UART_MUTEX_UNLOCK(&uart2);

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

    while(length--)
        serial2PutC(*ptr++);
}

int16_t static serial2Read (void)
{
    UART_MUTEX_LOCK(&uart2);

    int16_t data;
    uint16_t bptr = rxbuffer2.tail;

    if(bptr == rxbuffer2.head) {
        UART_MUTEX_UNLOCK(&uart2);
        return -1; // no data available else EOF
    }

    data = rxbuffer2.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer2.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    UART_MUTEX_UNLOCK(&uart2);

    return data;
}

IRAM_ATTR static void serial2Flush (void)
{
    UART_MUTEX_LOCK(&uart2);

    _uart_flush(&uart2, false);

    rxbuffer2.tail = rxbuffer2.head;
    rxbuffer2.overflow = Off;

    UART_MUTEX_UNLOCK(&uart2);
}

IRAM_ATTR static void serial2TxFlush (void)
{
    UART_MUTEX_LOCK(&uart2);

    _uart_flush(&uart2, true);

    UART_MUTEX_UNLOCK(&uart2);
}

IRAM_ATTR static void serial2Cancel (void)
{
    UART_MUTEX_LOCK(&uart2);

    rxbuffer2.data[rxbuffer2.head] = ASCII_CAN;
    rxbuffer2.tail = rxbuffer2.head;
    rxbuffer2.head = (rxbuffer2.tail + 1) & (RX_BUFFER_SIZE - 1);

    UART_MUTEX_UNLOCK(&uart2);
}

static bool serial2SuspendInput (bool suspend)
{
    bool ok;

    UART_MUTEX_LOCK(&uart2);

    ok = stream_rx_suspend(&rxbuffer2, suspend);

    UART_MUTEX_UNLOCK(&uart2);

    return ok;
}

IRAM_ATTR static bool serial2Disable (bool disable)
{
    UART_MUTEX_LOCK(&uart2);

    uart_ll_disable_intr_mask(uart2.dev, rx_int_flags);

    if(!disable) {
        // Clear and enable interrupts
        _uart_flush(&uart2, false);
        rxbuffer2.tail = rxbuffer2.head;
        uart_ll_clr_intsts_mask(uart2.dev, rx_int_flags);
        uart_ll_ena_intr_mask(uart2.dev, rx_int_flags);
    }

    UART_MUTEX_UNLOCK(&uart2);

    return true;
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
    uartSetBaudRate(&uart2, baud_rate);

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

static const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .state.connected = true,
        .read = serial2Read,
        .write = serial2WriteS,
        .write_n =  serial2Write,
        .write_char = serial2PutC,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RXFree,
        .get_rx_buffer_count = serial2Available,
        .get_tx_buffer_count = serial2TxCount,
        .reset_write_buffer = serial2TxFlush,
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

    memcpy(&uart2, &_uart_bus_array[1], sizeof(uart_t)); // use UART 1

    uartConfig(&uart2, baud_rate);

    serial2Flush();
#ifdef UART2_TX_PIN
    uartEnableInterrupt(&uart2, _uart2_isr, true);
#else
    uartEnableInterrupt(&uart2, _uart2_isr, false);
#endif

    return &stream;
}

#endif // SERIAL2_ENABLE
