/*
  BlackBoxX32_map.h - grblHAL - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2022 Peter van der Walt

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

  Pin Definitions for the OpenBuilds BlackBox X32 Controller from https://openbuilds.com

*/

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "BlackBox X32"
#define BOARD_URL "https://docs.openbuilds.com/doku.php?id=docs:blackbox-x32:start"
#define HAS_IOPORTS

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

// Stepper Driver Pins
#define STEPPERS_ENABLE_PIN	  GPIO_NUM_17

#define X_STEP_PIN            GPIO_NUM_12
#define X_DIRECTION_PIN       GPIO_NUM_14

#define Y_STEP_PIN            GPIO_NUM_27
#define Y_DIRECTION_PIN       GPIO_NUM_26

#define Z_STEP_PIN            GPIO_NUM_15
#define Z_DIRECTION_PIN       GPIO_NUM_2

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
  #define M3_AVAILABLE
  #define M3_STEP_PIN         GPIO_NUM_33
  #define M3_DIRECTION_PIN    GPIO_NUM_32
 #if N_AUTO_SQUARED
  #define M3_LIMIT_PIN        GPIO_NUM_22
  #if PROBE_ENABLE
   #warning "Probe input is not available when an auto-squared axis is enabled."
   #undef PROBE_ENABLE
   #define PROBE_ENABLE 0
  #endif
 #endif
#endif

// Endstops
#define X_LIMIT_PIN           GPIO_NUM_35
#define Y_LIMIT_PIN           GPIO_NUM_34
#define Z_LIMIT_PIN           GPIO_NUM_39

// Define probe switch input pin.
// NOTE: probe input is not available when an auto-squared axis is enabled.
#if PROBE_ENABLE
  #define PROBE_PIN           GPIO_NUM_22
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN    GPIO_NUM_13
#define SPINDLEPWMPIN         GPIO_NUM_25
#define SPINDLE_DIRECTION_PIN GPIO_NUM_4

// Define flood and mist coolant enable output pins.
// Only one can be enabled!
#define COOLANT_FLOOD_PIN     GPIO_NUM_21 // coolant
//#define COOLANT_MIST_PIN      GPIO_NUM_21 // or mist

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#if SAFETY_DOOR_ENABLE
  #define SAFETY_DOOR_PIN     GPIO_NUM_16
#endif


#ifdef HAS_IOPORTS
#define AUXINPUT0_PIN         GPIO_NUM_0 // Mode button on front panel
#endif

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO          GPIO_NUM_19
#define PIN_NUM_MOSI          GPIO_NUM_23
#define PIN_NUM_CLK           GPIO_NUM_18
//
// #if SDCARD_ENABLE
#define PIN_NUM_CS            GPIO_NUM_5
// #endif

#if MODBUS_ENABLE
#error BlackBox X32 does not support Modbus
#endif

#if KEYPAD_ENABLE
#error BlackBox X32 does not support Keypad
#endif

#if TRINAMIC_ENABLE
#error BlackBox X32 does not use Trinamic drivers
#endif
