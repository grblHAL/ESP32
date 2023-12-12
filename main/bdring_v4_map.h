/*
  bdring_v4_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

#if N_ABC_MOTORS > 0
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "BDRING v4"

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

// Define step pulse output pins.
#define X_STEP_PIN          GPIO_NUM_12
#define Y_STEP_PIN          GPIO_NUM_26
#define Z_STEP_PIN          GPIO_NUM_27

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_14
#define Y_DIRECTION_PIN     GPIO_NUM_15
#define Z_DIRECTION_PIN     GPIO_NUM_33

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN GPIO_NUM_13

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN         GPIO_NUM_17
#define Y_LIMIT_PIN         GPIO_NUM_4
#define Z_LIMIT_PIN         GPIO_NUM_16

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE && !(MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define SPINDLE_PWM_PIN     GPIO_NUM_2
#endif

#if DRIVER_SPINDLE_ENABLE && !(MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define SPINDLE_ENABLE_PIN  GPIO_NUM_22
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_25
#if DRIVER_SPINDLE_ENABLE
#define COOLANT_MIST_PIN    GPIO_NUM_21
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           GPIO_NUM_34
#define FEED_HOLD_PIN       GPIO_NUM_36
#define CYCLE_START_PIN     GPIO_NUM_39
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     GPIO_NUM_35
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN           GPIO_NUM_32
#endif

#if SDCARD_ENABLE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO        GPIO_NUM_19
#define PIN_NUM_MOSI        GPIO_NUM_23
#define PIN_NUM_CLK         GPIO_NUM_18
#define PIN_NUM_CS          GPIO_NUM_5
#endif

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#define UART2_RX_PIN            GPIO_NUM_22
#define UART2_TX_PIN            GPIO_NUM_21
#if RS485_DIR_ENABLE
#define MODBUS_DIRECTION_PIN    GPIO_NUM_2
#endif
#endif

#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif
