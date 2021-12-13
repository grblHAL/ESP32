/*
  xPro_v5_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if VFD_SPINDLE
#error "BOARD_XPRO_V5 does not have support for VFD spindle."
#endif

#define BOARD_NAME "x-Pro v5"
#define HAS_BOARD_INIT
#if TRINAMIC_ENABLE != 5160
#error BOARD_XPRO_V5 has soldered TMC5160 drivers.
#endif

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

// Define step pulse output pins.
#define X_STEP_PIN          GPIO_NUM_12
#define Y_STEP_PIN          GPIO_NUM_27
#define Z_STEP_PIN          GPIO_NUM_15

// Define step direction output pins.
#define X_DIRECTION_PIN     GPIO_NUM_14
#define Y_DIRECTION_PIN     GPIO_NUM_26
#define Z_DIRECTION_PIN     GPIO_NUM_2

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN         GPIO_NUM_35
#define Y_LIMIT_PIN         GPIO_NUM_34
#define Z_LIMIT_PIN         GPIO_NUM_39

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         GPIO_NUM_33
#define M3_DIRECTION_PIN    GPIO_NUM_32
#define M3_LIMIT_PIN        GPIO_NUM_36
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN  GPIO_NUM_4
#define SPINDLEPWMPIN       GPIO_NUM_25

// Define flood and mist coolant enable output pins.
// Only one can be enabled!

#define COOLANT_MIST_PIN        GPIO_NUM_21
//#define COOLANT_FLOOD_PIN		GPIO_NUM_21

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
//#define RESET_PIN           GPIO_NUM_34
//#define FEED_HOLD_PIN       GPIO_NUM_36
//#define CYCLE_START_PIN     GPIO_NUM_39
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN     GPIO_NUM_16
#else
#define RESET_PIN           GPIO_NUM_16 //??
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN           GPIO_NUM_22
#endif

#if SDCARD_ENABLE || TRINAMIC_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO        GPIO_NUM_19
#define PIN_NUM_MOSI        GPIO_NUM_23
#define PIN_NUM_CLK         GPIO_NUM_18
#define PIN_NUM_CS          GPIO_NUM_5
#define MOTOR_CS_PIN        GPIO_NUM_17

#endif // SDCARD_ENABLE

#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif
