/*

  usb_serial.c - driver code for ESP32S3

  Part of grblHAL

  Some parts are copyright (c) 2023-2026 Terje Io

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

***

  Some parts of the code is lifted from the Pico SDK pico_stdio_usb library and are

  Copyright (c) 2020 Raspberry Pi (Trading) Ltd.

  SPDX-License-Identifier: BSD-3-Clause
  
*/

#include "driver.h"

#if CONFIG_IDF_TARGET_ESP32S3 && USB_SERIAL_CDC

#include <stdint.h>
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "esp_private/usb_phy.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/clk_gate_ll.h"
#include "hal/usb_phy_ll.h"
#include "hal/usb_serial_jtag_ll.h"
#include "soc/periph_defs.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/usb_pins.h"
#include "soc/usb_serial_jtag_reg.h"
#include "tinyusb.h"
#include "tusb_tasks.h"
#include "tusb_cdc_acm.h"
#include <string.h>

#include "usb_serial.h"
#include "grbl/protocol.h"

#define BLOCK_RX_BUFFER_SIZE 20

static stream_block_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf;
static volatile enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
static bool usb_serial_started = false;
static bool usb_bootloader_restart_pending = false;
static usb_phy_handle_t usb_serial_phy_hdl = NULL;

/*
 * ESP-IDF 4.4 exposes TinyUSB install but not uninstall. The stock driver keeps
 * the OTG PHY handle private, which prevents the ESP32-S3 bootloader handoff
 * used by Arduino from tearing the PHY down before switching to USB Serial/JTAG.
 *
 * This follows the Arduino-ESP32 native USB path:
 * - cores/esp32/esp32-hal-tinyusb.c: usb_switch_to_cdc_jtag()
 * - cores/esp32/USBCDC.cpp: usb_persist_restart(RESTART_BOOTLOADER)
 */
extern tusb_desc_device_t descriptor_kconfig;
extern tusb_desc_strarray_device_t descriptor_str_kconfig;
void tusb_set_descriptor (tusb_desc_device_t *desc, const char **str_desc);

static void hw_cdc_reset_handler (void *arg)
{
    BaseType_t xTaskWoken = pdFALSE;
    uint32_t usbjtag_intr_status = usb_serial_jtag_ll_get_intsts_mask();

    usb_serial_jtag_ll_clr_intsts_mask(usbjtag_intr_status);

    if(usbjtag_intr_status & USB_SERIAL_JTAG_INTR_BUS_RESET)
        xSemaphoreGiveFromISR((SemaphoreHandle_t)arg, &xTaskWoken);

    if(xTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

static void usb_serial_bootloader_shutdown_handler (void)
{
    if(!usb_bootloader_restart_pending)
        return;

    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
}

static esp_err_t usb_serial_driver_install (const tinyusb_config_t *config)
{
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE
    };
    tusb_desc_device_t *dev_descriptor;
    const char **string_descriptor;
    esp_err_t err;

    if(config == NULL)
        return ESP_ERR_INVALID_ARG;

    err = usb_new_phy(&phy_conf, &usb_serial_phy_hdl);
    if(err != ESP_OK)
        return err;

    dev_descriptor = config->descriptor ? config->descriptor : &descriptor_kconfig;
    string_descriptor = config->string_descriptor ? config->string_descriptor : descriptor_str_kconfig;

    tusb_set_descriptor(dev_descriptor, string_descriptor);

    if(!tusb_init()) {
        usb_del_phy(usb_serial_phy_hdl);
        usb_serial_phy_hdl = NULL;
        return ESP_FAIL;
    }

    err = tusb_run_task();

    if(err != ESP_OK) {
        usb_del_phy(usb_serial_phy_hdl);
        usb_serial_phy_hdl = NULL;
    }

    return err;
}

static void usb_serial_driver_uninstall (void)
{
    if(usb_serial_phy_hdl)
        tusb_stop_task();

    if(usb_serial_phy_hdl) {
        usb_del_phy(usb_serial_phy_hdl);
        usb_serial_phy_hdl = NULL;
    }
}

static inline bool usb_connected (void)
{
  return tud_ready();
}

static bool usb_is_connected(void)
{
    return tud_cdc_n_connected(0);
}

static void usb_out_chars (const uint8_t *buf, int length)
{
    if(usb_connected()) {
        for(int i = 0; i < length;) {
        	size_t n = length - i;
        	uint32_t avail = tud_cdc_n_write_available(TINYUSB_USBDEV_0);
            if(n > avail)
                n = avail;
            if(n) {
                size_t n2 = tinyusb_cdcacm_write_queue(TINYUSB_USBDEV_0, buf + i, n);
                tinyusb_cdcacm_write_flush(TINYUSB_USBDEV_0, 2);
                i += n2;
            } else if(tinyusb_cdcacm_write_flush(TINYUSB_USBDEV_0, 2) == ESP_ERR_TIMEOUT)
                break;
        }
    }
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
static void usb_serialWrite (const uint8_t *s, uint16_t length)
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
        usb_serialWrite((const uint8_t *)s, length);
}

//
// Writes a character to the USB output stream
//
static bool usb_serialPutC (const uint8_t c)
{
    static uint8_t s[2] = "";

    *s = c;

    if(txbuf.length)
        usb_serialWriteS((const char *)s);
    else
        usb_out_chars(s, 1);

    return true;
}

//
// serialGetC - returns -1 if no data available
//
static int32_t usb_serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;

    if(tail == rxbuf.head)
        return -1; // no data available

    int32_t data = (int32_t)rxbuf.data[tail];   // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(tail, rxbuf);          // and update pointer

    return data;
}

static bool usb_serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand (uint8_t c)
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

// never called?
void usb_line_state_callback (int itf, cdcacm_event_t *event)
{
    (void)itf;

    stream_usb_linestate_changed(0, (serial_linestate_t){ .dtr = event->line_state_changed_data.dtr, .rts = event->line_state_changed_data.rts });
}

bool usb_serialEnterBootloader (void)
{
    intr_handle_t intr_handle = NULL;
    SemaphoreHandle_t reset_sem = NULL;

    if(!usb_serial_started)
        return false;

    usb_serial_driver_uninstall();
    periph_ll_reset(PERIPH_USB_MODULE);
    periph_ll_disable_clk_set_rst(PERIPH_USB_MODULE);

    CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL | RTC_CNTL_SW_USB_PHY_SEL | RTC_CNTL_USB_PAD_ENABLE);
    CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PHY_SEL);
    CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);

    // Force a disconnect while re-routing the internal PHY from USB OTG to USB Serial/JTAG.
    gpio_set_direction(USBPHY_DM_NUM, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(USBPHY_DP_NUM, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(USBPHY_DM_NUM, 0);
    gpio_set_level(USBPHY_DP_NUM, 0);

    usb_phy_ll_int_jtag_enable(&USB_SERIAL_JTAG);
    usb_serial_jtag_ll_disable_intr_mask(USB_SERIAL_JTAG_LL_INTR_MASK);
    usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_LL_INTR_MASK);
    usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_BUS_RESET);

    reset_sem = xSemaphoreCreateBinary();
    if(reset_sem && esp_intr_alloc(ETS_USB_SERIAL_JTAG_INTR_SOURCE, 0, hw_cdc_reset_handler, reset_sem, &intr_handle) == ESP_OK) {
        SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
        xSemaphoreTake(reset_sem, pdMS_TO_TICKS(1000));
    } else
        SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);

    usb_serial_jtag_ll_disable_intr_mask(USB_SERIAL_JTAG_LL_INTR_MASK);

    if(intr_handle)
        esp_intr_free(intr_handle);

    if(reset_sem)
        vSemaphoreDelete(reset_sem);

    usb_bootloader_restart_pending = esp_register_shutdown_handler(usb_serial_bootloader_shutdown_handler) == ESP_OK;

    if(!usb_bootloader_restart_pending)
        REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);

    esp_restart();

    return true;
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
        .callback_line_state_changed = usb_line_state_callback,
        .callback_line_coding_changed = NULL
    };

    usb_serial_driver_install(&tusb_cfg);
    tusb_cdc_acm_init(&acm_cfg);
    usb_serial_started = true;

    txbuf.s = txbuf.data;
    txbuf.max_length = CFG_TUD_CDC_TX_BUFSIZE;
    txbuf.max_length = (txbuf.max_length > BLOCK_TX_BUFFER_SIZE ? BLOCK_TX_BUFFER_SIZE : txbuf.max_length) - 20;

    return &stream;
}

#endif // GRBL_ESP32S3
