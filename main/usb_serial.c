/*

  usb_serial.c - driver code for ESP32S3

  Part of grblHAL

  Some parts are copyright (c) 2021-2023 Terje Io

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

***

  Some parts of the code is lifted from the Pico SDK pico_stdio_usb library and are

  Copyright (c) 2020 Raspberry Pi (Trading) Ltd.

  SPDX-License-Identifier: BSD-3-Clause
  
*/

#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#include <string.h>

#include "usb_serial.h"
#include "driver.h"
#include "grbl/protocol.h"

//#if USB_SERIAL_CDC == 2

#define BLOCK_RX_BUFFER_SIZE 20

static stream_block_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf;
static volatile enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static inline bool usb_connected (void)
{
  return tud_ready();
}

static bool usb_is_connected(void)
{
    return tud_cdc_n_connected(0);
}

static void usb_out_chars (const char *buf, int length)
{
    if(usb_connected()) {
        for(int i = 0; i < length;) {
        	size_t n = length - i;
        	uint32_t avail = tud_cdc_n_write_available(TINYUSB_USBDEV_0);
            if(n > avail)
                n = avail;
            if(n) {
                size_t n2 = tinyusb_cdcacm_write_queue(TINYUSB_USBDEV_0, (uint8_t *)buf + i, n);
                tinyusb_cdcacm_write_flush(TINYUSB_USBDEV_0, 2);
                i += n2;
            } else if(tinyusb_cdcacm_write_flush(TINYUSB_USBDEV_0, 2) == ESP_ERR_TIMEOUT)
                break;
        }
    }
}

static int32_t usb_in_chars (char *buf, uint32_t length)
{
    uint32_t count = 0;

    if (usb_connected() && tud_cdc_available())
            count = tud_cdc_read(buf, length);

    return count ? count : -1;
}

//
// Returns number of characters in USB input buffer
//
static uint16_t usb_serialRxCount (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;

    return (uint16_t)BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in USB input buffer
//
static uint16_t usb_serialRxFree (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;
 
    return (uint16_t)((RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE));
}

//
// Flushes the USB input buffer (including the USB buffer)
//
static void usb_serialRxFlush (void)
{
  //  usb_serial_flush_input();
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the USB input buffer
//
static void usb_serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = CMD_RESET;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the USB output stream
//
static bool usb_serialPutC (const char c)
{
    static char buf[1];

    *buf = c;
    usb_out_chars(buf, 1);

    return true;
}

bool _usb_write (void)
{
    size_t txfree, length;

    txbuf.s = txbuf.data;

    while(txbuf.length) {

        if((txfree = tud_cdc_write_available()) > 10) {

            length = txfree < txbuf.length ? txfree : txbuf.length;

            usb_out_chars(txbuf.s, length); //

            txbuf.length -= length;
            txbuf.s += length;
        }

        if(txbuf.length && !hal.stream_blocking_callback()) {
            txbuf.length = 0;
            txbuf.s = txbuf.data;
            return false;
        }
    }

    txbuf.s = txbuf.data;

    return true;
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void usb_serialWrite (const char *s, uint16_t length)
{
    // Empty buffer first...
    if(txbuf.length && !_usb_write())
        return;

    usb_out_chars(s, length);
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
//
static void usb_serialWriteS (const char *s)
{
    if(*s == '\0')
        return;

    size_t length = strlen(s);

    if((length + txbuf.length) < BLOCK_TX_BUFFER_SIZE) {

        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;

        if(s[length - 1] == ASCII_LF || txbuf.length > txbuf.max_length) {
            if(!_usb_write())
                return;
        }
    } else
        usb_serialWrite(s, length);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t usb_serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(tail, rxbuf); // and update pointer

    return (int16_t)data;
}

static bool usb_serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usb_serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static void usb_rx_callback (int itf, cdcacm_event_t *event)
{
    static uint8_t tmpbuf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];

    uint8_t c, *dp;
    size_t avail, free;

	dp = tmpbuf;
	free = (int32_t)usb_serialRxFree();
	free = free > CONFIG_TINYUSB_CDC_RX_BUFSIZE ? CONFIG_TINYUSB_CDC_RX_BUFSIZE : free;
	if(tinyusb_cdcacm_read(itf, tmpbuf, free, &avail) == ESP_OK) {
        if(avail > 0) while(avail--) {
            c = *dp++;
            if(!enqueue_realtime_command(c)) {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get next head pointer
                if(next_head == rxbuf.tail)                             // If buffer full
                    rxbuf.overflow = On;                                // flag overflow,
                else {
                    rxbuf.data[rxbuf.head] = c;                         // else add character data to buffer
                    rxbuf.head = next_head;                             // and update pointer
                }
            }
        }
	}
}

const io_stream_t *usb_serialInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .is_connected = usb_is_connected,
        .read = usb_serialGetC,
        .write = usb_serialWriteS,
        .write_n = usb_serialWrite,
        .write_char = usb_serialPutC,
        .enqueue_rt_command = usbEnqueueRtCommand,
        .get_rx_buffer_free = usb_serialRxFree,
        .get_rx_buffer_count = usb_serialRxCount,
        .reset_read_buffer = usb_serialRxFlush,
        .cancel_read_buffer = usb_serialRxCancel,
        .suspend_read = usb_serialSuspendInput,
        .set_enqueue_rt_handler = usb_serialSetRtHandler
    };

    tinyusb_config_t tusb_cfg = {};
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &usb_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    tinyusb_driver_install(&tusb_cfg);
    tusb_cdc_acm_init(&acm_cfg);

    txbuf.s = txbuf.data;
    txbuf.max_length = CFG_TUD_CDC_TX_BUFSIZE;
    txbuf.max_length = (txbuf.max_length > BLOCK_TX_BUFFER_SIZE ? BLOCK_TX_BUFFER_SIZE : txbuf.max_length) - 20;

    return &stream;
}

//
// This function get called from the foreground process,
// used here to get characters off the USB serial input stream and buffer
// them for processing by grbl. Real time command characters are stripped out
// and submitted for realtime processing.
//
static void execute_realtime (uint_fast16_t state)
{
    static volatile bool lock = false;
    static char tmpbuf[BLOCK_RX_BUFFER_SIZE];

    if(lock)
        return;

    char c, *dp;
    int32_t avail, free;
 
    lock = true;
 
    if(usb_connected() && (avail = (int32_t)tud_cdc_available())) {

        dp = tmpbuf;
        free = (int32_t)usb_serialRxFree();
        free = free > BLOCK_RX_BUFFER_SIZE ? BLOCK_RX_BUFFER_SIZE : free;
        avail = usb_in_chars(tmpbuf, (uint32_t)(avail > free ? free : avail));

        if(avail > 0) while(avail--) {
            c = *dp++;
            if(!enqueue_realtime_command(c)) {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get next head pointer
                if(next_head == rxbuf.tail)                             // If buffer full
                    rxbuf.overflow = On;                                // flag overflow,
                else {
                    rxbuf.data[rxbuf.head] = c;                         // else add character data to buffer
                    rxbuf.head = next_head;                             // and update pointer
                }
            }
        }
    }

    lock = false;
}
