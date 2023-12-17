/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2018-2023 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef __DRIVER_H__
#define __DRIVER_H__

#ifndef OVERRIDE_MY_MACHINE
//
// Set options from my_machine.h
//
#include "my_machine.h"

#if WEBUI_ENABLE
#error "WebUI is not available in this setup!"
#endif
//
#else
//
// Process options from CMakeLists.txt
//

#if WIFI_ENABLE || ETHERNET_ENABLE

//#define TELNET_ENABLE           1
//#define WEBSOCKET_ENABLE        1
//#define NETWORK_TELNET_PORT     23
//#define NETWORK_FTP_PORT        21
//#define NETWORK_HTTP_PORT       80
//#define NETWORK_WEBSOCKET_PORT  81

// Ethernet settings
//#define NETWORK_HOSTNAME        "grblHAL"
//#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_IP              "192.168.5.1"
//#define NETWORK_GATEWAY         "192.168.5.1"
//#define NETWORK_MASK            "255.255.255.0"

// WiFi Station (STA) settings
//#define NETWORK_STA_HOSTNAME    "grblHAL"
//#define NETWORK_STA_IPMODE      1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_STA_IP          "192.168.5.1"
//#define NETWORK_STA_GATEWAY     "192.168.5.1"
//#define NETWORK_STA_MASK        "255.255.255.0"

// WiFi Access Point (AP) settings
#if WIFI_SOFTAP
//#define NETWORK_AP_HOSTNAME     "grblHAL_AP"
//#define NETWORK_AP_IP           "192.168.5.1"
//#define NETWORK_AP_GATEWAY      "192.168.5.1"
//#define NETWORK_AP_MASK         "255.255.255.0"
//#define NETWORK_AP_SSID         "grblHAL_AP"
//#define NETWORK_AP_PASSWORD     "grblHALpwd" // Minimum 8 characters, or blank for open
#define WIFI_MODE WiFiMode_AP; // OPTION: WiFiMode_APSTA
#else
#define WIFI_MODE WiFiMode_STA; // Do not change!
#endif

#endif // WIFI_ENABLE || ETHERNET_ENABLE

#ifndef RS485_DIR_ENABLE
#define RS485_DIR_ENABLE 0
#endif

#endif // CMakeLists options

#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/i2c.h"
#include "hal/gpio_types.h"
#include "esp_log.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "grbl/hal.h"
#include "grbl/driver_opts.h"

#if WIFI_ENABLE && NETWORK_STA_IPMODE == 0 && WIFI_SOFTAP
#error "Cannot use static IP for station when soft AP is enabled!"
#endif

#ifndef GRBLHAL_TASK_PRIORITY
#define GRBLHAL_TASK_PRIORITY 5
#endif
#ifndef GRBLHAL_TASK_CORE
#define GRBLHAL_TASK_CORE 1
#endif

#define PROBE_ISR 0 // Catch probe state change by interrupt TODO: needs verification!

// DO NOT change settings here!

#ifndef IOEXPAND_ENABLE
#define IOEXPAND_ENABLE 0 // I2C IO expander for some output signals.
#endif

#define IOEXPAND 0xFF   // Dummy pin number for I2C IO expander

// end configuration

#if !(WIFI_ENABLE || ETHERNET_ENABLE) && (HTTP_ENABLE || TELNET_ENABLE || WEBSOCKET_ENABLE || FTP_ENABLE)
#error "Networking protocols requires networking enabled!"
#endif

// End configuration

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

// TODO: move to wifi.c!
typedef struct
{
    grbl_wifi_mode_t mode;
    wifi_sta_settings_t sta;
    wifi_ap_settings_t ap;
} wifi_settings_t;

typedef struct {
    uint8_t action;
    uint_fast16_t address;
    void *params;
} i2c_task_t;

// End configuration

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_BDRING_V4)
  #include "bdring_v4_map.h"
#elif defined(BOARD_BDRING_V3P5)
  #include "bdring_v3.5_map.h"
#elif defined(BOARD_BDRING_I2S6A)
  #include "bdring_i2s_6_axis_map.h"
#elif defined(BOARD_ESPDUINO32)
  #include "espduino-32_wemos_d1_r32_uno_map.h"
#elif defined(BOARD_SOURCERABBIT_4AXIS)
  #include "sourcerabbit_4axis.h"
#elif defined(BOARD_PROTONEER_3XX)
  #include "protoneer_3.xx_map.h"
#elif defined(BOARD_FYSETC_E4)
  #include "fysetc_e4_map.h"
#elif defined(BOARD_XPRO_V5)
  #include "xPro_v5_map.h"
#elif defined(BOARD_MKS_DLC32_V2P0)
  #include "mks_dlc32_2_0_map.h"
#elif defined(BOARD_MKS_TINYBEE_V1)
  #include "mks_tinybee_1_0_map.h"
#elif defined(BOARD_BLACKBOX_X32)
  #include "BlackBoxX32_map.h"
#elif defined(BOARD_ROOTCNC_V2)
  #include "root_cnc_v2_map.h"
#elif defined(BOARD_ROOTCNC_V3)
  #include "root_cnc_v3_map.h"
#elif defined(BOARD_BLOX)
  #include "blox_map.h"
#elif defined(BOARD_CNC3040)
  #include "cnc3040_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else // default board - NOTE: NOT FINAL VERSION!
  #warning "Compiling for generic board!"
  #include "generic_map.h"
#endif

#ifndef GRBL_ESP32
#error "Add #define GRBL_ESP32 in grbl/config.h or update your CMakeLists.txt to the latest version!"
#endif

#if IOEXPAND_ENABLE == 0 && ((DIRECTION_MASK|STEPPERS_DISABLE_MASK|SPINDLE_MASK|COOLANT_MASK) & 0xC00000000ULL)
#error "Pins 34 - 39 are input only!"
#endif

#if DRIVER_SPINDLE_PWM_ENABLE && !defined(SPINDLE_PWM_PIN)
#warning "PWM spindle is not supported by board map!"
#undef DRIVER_SPINDLE_PWM_ENABLE
#define DRIVER_SPINDLE_PWM_ENABLE 0
#endif

#if IOEXPAND_ENABLE || EEPROM_ENABLE || KEYPAD_ENABLE == 1 || I2C_STROBE_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
#undef I2C_ENABLE
#define I2C_ENABLE 1
#elif !defined(I2C_ENABLE)
#define I2C_ENABLE 0
#endif

#ifdef I2C_PORT
extern QueueHandle_t i2cQueue;
extern SemaphoreHandle_t i2cBusy;
#elif I2C_ENABLE == 1
#error "I2C port not available!"
#endif

#if USB_SERIAL_CDC
#define SP0 1
#else
#define SP0 0
#endif

#ifdef UART2_RX_PIN
#define SP1 1
#else
#define SP1 0
#endif

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#define MODBUS_TEST 1
#else
#define MODBUS_TEST 0
#endif

#if TRINAMIC_UART_ENABLE
#define TRINAMIC_TEST 1
#else
#define TRINAMIC_TEST 0
#endif

#if MPG_ENABLE
#define MPG_TEST 1
#else
#define MPG_TEST 0
#endif

#if KEYPAD_ENABLE == 2 && MPG_ENABLE == 0
#define KEYPAD_TEST 1
#else
#define KEYPAD_TEST 0
#endif

#if (MODBUS_TEST + KEYPAD_TEST + MPG_TEST + TRINAMIC_TEST) > (SP0 + SP1)
#error "Too many options that requires a serial port are enabled!"
#elif (MODBUS_TEST + KEYPAD_TEST + MPG_TEST + TRINAMIC_TEST)
#define SERIAL2_ENABLE 1
#else
#define SERIAL2_ENABLE 0
#endif

#undef SP0
#undef SP1
#undef MODBUS_TEST
#undef KEYPAD_TEST
#undef MPG_TEST
#undef TRINAMIC_TEST

#if MPG_MODE == 1
  #ifndef MPG_ENABLE_PIN
  #error "MPG_ENABLE_PIN must be defined when MPG mode is enabled!"
  #endif
  #ifndef UART2_RX_PIN
  #error "UART2_RX_PIN must be defined when MPG mode is enabled!"
  #endif
#endif

#ifndef I2S_OUT_PIN_BASE
#define I2S_OUT_PIN_BASE 64
#endif

#ifdef USE_I2S_OUT
#undef USE_I2S_OUT
#define USE_I2S_OUT 1
#define DIGITAL_IN(pin) i2s_out_state(pin - I2S_OUT_PIN_BASE)
#define DIGITAL_OUT(pin, state) i2s_out_write(pin - I2S_OUT_PIN_BASE, state)
#else
#define USE_I2S_OUT 0
#define DIGITAL_IN(pin) gpio_get_level(pin)
#define DIGITAL_OUT(pin, state) gpio_set_level(pin, state)
#endif

typedef enum
{
    Pin_GPIO = 0,
    Pin_RMT,
    Pin_IoExpand,
    Pin_I2S
} esp_pin_t;

typedef struct {
    pin_function_t id;
    pin_group_t group;
    uint8_t pin;
    uint32_t mask;
    uint8_t offset;
    bool invert;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    volatile bool active;
    volatile bool debounce;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    pin_group_t group;
    uint8_t pin;
    esp_pin_t mode;
    bool claimed;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

gpio_int_type_t map_intr_type (pin_irq_mode_t mode);
void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#ifdef HAS_BOARD_INIT
void board_init (void);
#endif

#endif // __DRIVER_H__
