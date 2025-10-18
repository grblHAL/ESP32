/*
  mks_dlc32_max_1_0_map.h.h - driver code for ESP32

  Part of grblHAL

  Copyright (c) 2025 @luc-github

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

#ifndef CONFIG_IDF_TARGET_ESP32S3
#error "This board has ESP32-S3 processor, select a corresponding build!"
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if VFD_SPINDLE
#error "Generic map does not have support for VFD spindle."
#endif

#if KEYPAD_ENABLE == 1
#error No free pins for I2C keypad!
#endif

#define BOARD_NAME "MKS DLC32 MAX 1.0"
#define BOARD_URL "https://github.com/makerbase-mks/MKS-DLC32"

#undef I2C_ENABLE
#define I2C_ENABLE 0

#define SERIAL1_PORT 
#define UART1_RX_PIN            GPIO_NUM_18
#define UART1_TX_PIN            GPIO_NUM_17

// Define step pulse output pins.
#define X_STEP_PIN              GPIO_NUM_16
#define Y_STEP_PIN              GPIO_NUM_7
#define Z_STEP_PIN              GPIO_NUM_5

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN         GPIO_NUM_15
#define Y_DIRECTION_PIN         GPIO_NUM_6
#define Z_DIRECTION_PIN         GPIO_NUM_4

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN     GPIO_NUM_8

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN             GPIO_NUM_39 
#define Y_LIMIT_PIN             GPIO_NUM_40 
#define Z_LIMIT_PIN             GPIO_NUM_41 

#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PIN             GPIO_NUM_20
#define M3_DIRECTION_PIN        GPIO_NUM_19
#define M3_LIMIT_PIN            GPIO_NUM_48
#endif

#define AUXOUTPUT0_PIN          GPIO_NUM_2  // Laser TTL
#define AUXOUTPUT1_PIN          GPIO_NUM_35 // Spindle enable
#define AUXOUTPUT2_PIN          GPIO_NUM_1  // Air pump
#define AUXOUTPUT3_PIN          GPIO_NUM_41 // Beeper
//#define AUXOUTPUT4_PIN          GPIO_NUM_? // TBD

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN //Laser TTL header
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT1_PIN //Spindle enable
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT2_PIN //Air pump header
#endif

#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT3_PIN //Beeper header
#endif

#if SDCARD_ENABLE

#define SDCARD_SDIO         1
#define SD_CLK_PIN          GPIO_NUM_13  //SD card clock
#define SD_CMD_PIN          GPIO_NUM_14  //SD card command
#define SD_DETECT_PIN       GPIO_NUM_10  //SD card detect
#define SD_D0_PIN           GPIO_NUM_12  //SD card data 0
#define SD_D1_PIN           GPIO_NUM_11  //SD card data 1
#define SD_D2_PIN           GPIO_NUM_47  //SD card data 2
#define SD_D3_PIN           GPIO_NUM_21  //SD card data 3

#endif // SDCARD_ENABLE

#define AUXINPUT0_PIN           GPIO_NUM_38 // Door
#define AUXINPUT1_PIN           GPIO_NUM_37 // Probe
#define AUXINPUT2_PIN           GPIO_NUM_36 // Cycle start -> Flame

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT2_PIN
#endif

#if RGB_LED_ENABLE
#define LED_PIN                 GPIO_NUM_48 // for ESP32-S3-DevKit
#endif

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#endif
