/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2018-2024 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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
*/

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "sdkconfig.h"

#ifndef OVERRIDE_MY_MACHINE

#include "my_machine.h"

#if WEBUI_ENABLE && !defined(WIFI_ENABLE) && !defined(ETHERNET_ENABLE)
#define WIFI_ENABLE 1
#endif

#endif // OVERRIDE_MY_MACHINE

#if WEBUI_ENABLE && !defined(WEBUI_INFLASH)
#define WEBUI_INFLASH 1
#endif

#define EXPANDER_PORT 1

#define OPTS_POSTPROCESSING

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

#if PPI_ENABLE
#error "PPI mode not supported due to framework not supporting FPU access in interrupt context!"
#endif

#ifndef GRBLHAL_TASK_PRIORITY
#define GRBLHAL_TASK_PRIORITY 5
#endif

#ifndef GRBLHAL_TASK_CORE
#define GRBLHAL_TASK_CORE 1
#endif

#define PROBE_ISR 0 // Catch probe state change by interrupt TODO: needs verification!

// DO NOT change settings here!

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

typedef struct {
    uint8_t action;
    uint_fast16_t address;
    void *params;
} i2c_task_t;

#ifndef CONTROL_ENABLE
#define CONTROL_ENABLE (CONTROL_HALT|CONTROL_FEED_HOLD|CONTROL_CYCLE_START)
#endif

#if (MODBUS_ENABLE & MODBUS_RTU_ENABLED) || TRINAMIC_UART_ENABLE==1 || MPG_ENABLE || (KEYPAD_ENABLE == 2 && MPG_ENABLE == 0)
#define ADD_SERIAL1
#endif

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
#elif defined(BOARD_BDRING_6X_V3)
  #include "boards/bdring_i2s_6x_v3_map.h"
#elif defined(BOARD_BDRING_I2S_6PACK_EXT_V2)
  #include "boards/bdring_i2s_6pack_ext_v2_map.h"
#elif defined(BOARD_ESPDUINO32)
  #include "boards/espduino-32_wemos_d1_r32_uno_map.h"
#elif defined(BOARD_SOURCERABBIT_4AXIS) || defined(BOARD_SOURCERABBIT_4AXIS_12)
  #include "boards/sourcerabbit_4axis.h"
#elif defined(BOARD_PROTONEER_3XX)
  #include "boards/protoneer_3.xx_map.h"
#elif defined(BOARD_FYSETC_E4)
  #include "boards/fysetc_e4_map.h"
#elif defined(BOARD_XPRO_V5)
  #include "boards/xPro_v5_map.h"
#elif defined(BOARD_MKS_DLC32_V2P0)
  #include "boards/mks_dlc32_2_0_map.h"
#elif defined(BOARD_MKS_DLC32_MAX_V1)
  #include "boards/mks_dlc32_max_1_0_map.h"
#elif defined(BOARD_MKS_TINYBEE_V1)
  #include "boards/mks_tinybee_1_0_map.h"
#elif defined(BOARD_PIBOT_I2S_6_AXIS)
  #include "boards/pibot_i2s_6_axis_map.h"
#elif defined(BOARD_BLACKBOX_X32)
  #include "boards/BlackBoxX32_map.h"
#elif defined(BOARD_ROOTCNC_V2)
  #include "boards/root_cnc_v2_map.h"
#elif defined(BOARD_ROOTCNC_V3)
  #include "boards/root_cnc_v3_map.h"
#elif defined(BOARD_ROOTCNC_PRO)
  #include "boards/root_cnc_pro_map.h"
#elif defined(BOARD_CNC3040)
  #include "boards/cnc3040_map.h"
#elif defined(BOARD_JACKPOT)
  #include "boards/jackpot_map.h"
#elif defined(BOARD_BTT_RODENT)
  #include "boards/btt_rodent_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#elif defined(BOARD_GENERIC_S3)
  #include "boards/generic_s3_map.h"
#elif defined(BOARD_GENERIC_I2S_S3)
  #include "boards/generic_i2s_s3_map.h"
#else // default board - NOTE: NOT FINAL VERSION!
 #ifndef WEB_BUILD
  #warning "Compiling for generic board!"
 #endif
  #include "boards/generic_map.h"
#endif

#ifndef GRBL_ESP32
#error "Add #define GRBL_ESP32 in grbl/config.h or update your CMakeLists.txt to the latest version!"
#endif

#if ((DIRECTION_MASK|STEPPERS_DISABLE_MASK|SPINDLE_MASK|COOLANT_MASK) & 0xC00000000ULL)
#error "Pins 34 - 39 are input only!"
#endif

#if defined(USE_I2S_OUT) && STEP_INJECT_ENABLE
#error "Step injection not yet possible with I2S streaming!"
#endif

#if DRIVER_SPINDLE_PWM_ENABLE && !defined(SPINDLE_PWM_PIN)
#warning "PWM spindle is not supported by board map!"
#undef DRIVER_SPINDLE_PWM_ENABLE
#define DRIVER_SPINDLE_PWM_ENABLE 0
#endif

#if SAFETY_DOOR_ENABLE && !defined(SAFETY_DOOR_PIN)
#warning "Safety door input is not available!"
#undef SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_ENABLE 0
#endif

// Minimum pulse off time
#ifndef STEP_PULSE_TOFF_MIN
#define STEP_PULSE_TOFF_MIN 2.0f
#endif

#ifndef STEP_TIMER_GROUP
#define STEP_TIMER_GROUP TIMER_GROUP_0
#endif
#ifndef STEP_TIMER_INDEX
#define STEP_TIMER_INDEX TIMER_0
#endif

#if EEPROM_ENABLE || KEYPAD_ENABLE == 1 || I2C_STROBE_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
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

#if SDCARD_ENABLE && defined(SDCARD_SDIO) && SPI_ENABLE && !TRINAMIC_SPI_ENABLE
#undef SPI_ENABLE
#define SPI_ENABLE 0
#endif

// NOTE: #define SERIAL_PORT in map file if USB_SERIAL_CDC is enabled and the primary UART is not connected to a USB <> UART chip

#include "grbl/driver_opts2.h"

#if MPG_ENABLE == 1
  #ifndef MPG_ENABLE_PIN
  #error "MPG_ENABLE_PIN must be defined when MPG mode is enabled!"
  #endif
  #ifndef UART1_RX_PIN
  #error "UART1_RX_PIN must be defined when MPG mode is enabled!"
  #endif
#endif

#ifndef I2S_OUT_PIN_BASE
#define I2S_OUT_PIN_BASE 64
#endif

#ifndef USE_I2S_OUT
#define USE_I2S_OUT 0
#define DIGITAL_IN(pin) gpio_ll_get_level(&GPIO, pin)
#define DIGITAL_OUT(pin, state) gpio_ll_set_level(&GPIO, pin, (state))
#endif

#define EXPANDER_IN(pin) ( iox_out[pin] && iox_out[pin]->get_value(iox_out[pin]) != 0.0f )
#define EXPANDER_OUT(pin, state) { if(iox_out[pin]) iox_out[pin]->set_value(iox_out[pin], (float)state); }

#ifdef MODBUS_DIRECTION_PIN
#define MODBUS_DIR_AUX 0
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
    uint8_t pin;
    uint8_t user_port;
    uint8_t offset;
    pin_group_t group;
    void *port;
    uint32_t mask;
    pin_cap_t cap;
    pin_mode_t mode;
    pin_function_t id;
    const adc_map_t *adc;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    pin_group_t group;
    void *port;
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
