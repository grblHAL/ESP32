/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2018-2024 Terje Io
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

#include "my_machine.h"

#if WEBUI_ENABLE && !defined(WIFI_ENABLE) && !defined(ETHERNET_ENABLE)
#define WIFI_ENABLE 1
#endif

#endif // OVERRIDE_MY_MACHINE

#if WEBUI_ENABLE && !defined(WEBUI_INFLASH)
#define WEBUI_INFLASH 1
#endif

#include "grbl/driver_opts.h"

#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "hal/gpio_ll.h"
#include "esp_log.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "grbl/hal.h"

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

static const DRAM_ATTR float FZERO = 0.0f;

// end configuration

#if !(WIFI_ENABLE || ETHERNET_ENABLE) && (HTTP_ENABLE || TELNET_ENABLE || WEBSOCKET_ENABLE || FTP_ENABLE)
#error "Networking protocols requires networking enabled!"
#endif

#if WIFI_ENABLE
// WiFi Access Point (AP) settings
#if WIFI_SOFTAP
#define WIFI_MODE WiFiMode_AP; // OPTION: WiFiMode_APSTA
#else
#define WIFI_MODE WiFiMode_STA; // Do not change!
#endif
#endif // WIFI_ENABLE

// End configuration

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

typedef struct {
    uint8_t action;
    uint_fast16_t address;
    void *params;
} i2c_task_t;

// End configuration

#ifdef BOARD_CNC_BOOSTERPACK
  #include "boards/cnc_boosterpack_map.h"
#elif defined(BOARD_BDRING_V4)
  #include "boards/bdring_v4_map.h"
#elif defined(BOARD_BDRING_V3P5)
  #include "boards/bdring_v3.5_map.h"
#elif defined(BOARD_BDRING_I2S6A)
  #include "boards/bdring_i2s_6_axis_map.h"
#elif defined(BOARD_BDRING_6X)
  #include "boards/bdring_i2s_6x_v1_map.h"
#elif defined(BOARD_ESPDUINO32)
  #include "boards/espduino-32_wemos_d1_r32_uno_map.h"
#elif defined(BOARD_SOURCERABBIT_4AXIS)
  #include "boards/sourcerabbit_4axis.h"
#elif defined(BOARD_PROTONEER_3XX)
  #include "boards/protoneer_3.xx_map.h"
#elif defined(BOARD_FYSETC_E4)
  #include "boards/fysetc_e4_map.h"
#elif defined(BOARD_XPRO_V5)
  #include "boards/xPro_v5_map.h"
#elif defined(BOARD_MKS_DLC32_V2P0)
  #include "boards/mks_dlc32_2_0_map.h"
#elif defined(BOARD_MKS_TINYBEE_V1)
  #include "boards/mks_tinybee_1_0_map.h"
#elif defined(BOARD_BLACKBOX_X32)
  #include "boards/BlackBoxX32_map.h"
#elif defined(BOARD_ROOTCNC_V2)
  #include "boards/root_cnc_v2_map.h"
#elif defined(BOARD_ROOTCNC_V3)
  #include "boards/root_cnc_v3_map.h"
#elif defined(BOARD_BLOX)
  #include "boards/blox_map.h"
#elif defined(BOARD_CNC3040)
  #include "boards/cnc3040_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#elif defined(BOARD_GENERIC_I2S_S3)
  #include "boards/generic_i2s_s3_map.h"
#else // default board - NOTE: NOT FINAL VERSION!
  #warning "Compiling for generic board!"
  #include "boards/generic_map.h"
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

#ifdef UART3_RX_PIN
#define SP2 1
#else
#define SP2 0
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

#if (MODBUS_TEST + KEYPAD_TEST + MPG_TEST + TRINAMIC_TEST + (DEBUGOUT ? 1 : 0)) > (SP0 + SP1 + SP2)
#error "Too many options that requires a serial port are enabled!"
#elif (MODBUS_TEST + KEYPAD_TEST + MPG_TEST + TRINAMIC_TEST + DEBUGOUT)
#define SERIAL2_ENABLE 1
#else
#define SERIAL2_ENABLE 0
#endif

#undef SP0
#undef SP1
#undef SP2
#undef MODBUS_TEST
#undef KEYPAD_TEST
#undef MPG_TEST
#undef TRINAMIC_TEST

#if MPG_ENABLE
#if MPG_STREAM == 0
#define MPG_STREAM_DUPLEX 1
#elif MPG_STREAM == 1
#ifdef UART2_TX_PIN
#define MPG_STREAM_DUPLEX 1
#else
#define MPG_STREAM_DUPLEX 0
#endif
#elif MPG_STREAM == 2
#ifdef UART3_TX_PIN
#define MPG_STREAM_DUPLEX 1
#else
#define MPG_STREAM_DUPLEX 0
#endif
#endif
#endif

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
#define DIGITAL_IN(pin) (pin >= I2S_OUT_PIN_BASE ? i2s_out_state(pin - I2S_OUT_PIN_BASE) : gpio_ll_get_level(&GPIO, pin))
#define DIGITAL_OUT(pin, state) { if(pin >= I2S_OUT_PIN_BASE) i2s_out_write(pin - I2S_OUT_PIN_BASE, state); else gpio_ll_set_level(&GPIO, pin, state); }
#else
#define USE_I2S_OUT 0
#define DIGITAL_IN(pin) gpio_ll_get_level(&GPIO, pin)
#define DIGITAL_OUT(pin, state) gpio_ll_set_level(&GPIO, pin, state)
#endif

typedef enum
{
    Pin_GPIO = 0,
    Pin_RMT,
    Pin_IoExpand,
    Pin_I2S
} esp_pin_t;

typedef struct {
    adc1_channel_t ch;
    gpio_num_t pin;
} adc_map_t;

typedef struct {
    pin_function_t id;
    pin_group_t group;
    uint8_t pin;
    uint32_t mask;
    uint8_t offset;
    bool invert;
    volatile bool active;
    volatile bool debounce;
    pin_cap_t cap;
    pin_mode_t mode;
    const adc_map_t *adc;
    ioport_interrupt_callback_ptr interrupt_callback;
    aux_ctrl_t *aux_ctrl;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    pin_group_t group;
    uint8_t pin;
    esp_pin_t type;
    pin_mode_t mode;
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
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);

#ifdef HAS_BOARD_INIT
void board_init (void);
#endif

#endif // __DRIVER_H__
