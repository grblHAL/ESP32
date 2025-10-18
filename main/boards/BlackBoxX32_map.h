/*
  BlackBoxX32_map.h - grblHAL - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2022 Peter van der Walt

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

  Pin Definitions for the OpenBuilds BlackBox X32 Controller from https://openbuilds.com

*/

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if KEYPAD_ENABLE == 1
#error No free pins for I2C keypad!
#endif

#if TRINAMIC_ENABLE
#error BlackBox X32 does not use Trinamic drivers
#endif

#define BOARD_NAME "BlackBox X32"
#define BOARD_URL "https://docs.openbuilds.com/doku.php?id=docs:blackbox-x32:start"
#if N_AUTO_SQUARED || N_AXIS > 3
#define HAS_BOARD_INIT
#endif

// Stepper Driver Pins
#define STEPPERS_ENABLE_PIN GPIO_NUM_17

#define X_STEP_PIN          GPIO_NUM_12
#define X_DIRECTION_PIN     GPIO_NUM_14

#define Y_STEP_PIN          GPIO_NUM_27
#define Y_DIRECTION_PIN     GPIO_NUM_26

#define Z_STEP_PIN          GPIO_NUM_15
#define Z_DIRECTION_PIN     GPIO_NUM_2

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
  #define M3_AVAILABLE
  #define M3_STEP_PIN       GPIO_NUM_33
  #define M3_DIRECTION_PIN  GPIO_NUM_32
 #if N_AUTO_SQUARED
// add limit pin definitions to stop compiler complaints (from preprocessor).
  #if X_AUTO_SQUARE
   #define M3_LIMIT_PIN     GPIO_NUM_35 // Same as X limit, switched to Z by board code during homing.
  #elif Y_AUTO_SQUARE
   #define M3_LIMIT_PIN     GPIO_NUM_34 // Same as Y limit, switched to Z by board code during homing.
  #else
   #define M3_LIMIT_PIN     GPIO_NUM_39 // Same as Z limit, switched to X by board code during homing.
  #endif
 #endif
#endif

// Endstops
#define X_LIMIT_PIN         GPIO_NUM_35
#define Y_LIMIT_PIN         GPIO_NUM_34
#define Z_LIMIT_PIN         GPIO_NUM_39

#define AUXOUTPUT0_PIN      GPIO_NUM_25 // Spindle PWM
#define AUXOUTPUT1_PIN      GPIO_NUM_4  // Spindle direction
#define AUXOUTPUT2_PIN      GPIO_NUM_13 // Spindle enable
#define AUXOUTPUT3_PIN      GPIO_NUM_21 // Coolant flood

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#ifdef COOLANT_FLOOD_PIN
#define COOLANT_ENABLE COOLANT_FLOOD
#else
#define COOLANT_ENABLE 0
#endif
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0 // No control inputs

#define AUXINPUT0_PIN       GPIO_NUM_0 // Mode button on front panel
#define AUXINPUT1_PIN       GPIO_NUM_16
#define AUXINPUT2_PIN       GPIO_NUM_22

#if PROBE_ENABLE
 #define PROBE_PIN          AUXINPUT2_PIN
#endif

#if SAFETY_DOOR_ENABLE
  #define SAFETY_DOOR_PIN   AUXINPUT1_PIN
#endif

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define SPI_MISO_PIN        GPIO_NUM_19
#define SPI_MOSI_PIN        GPIO_NUM_23
#define SPI_SCK_PIN         GPIO_NUM_18
//
// #if SDCARD_ENABLE
#define SD_CS_PIN           GPIO_NUM_5
// #endif
