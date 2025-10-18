/*
  generic_s3_map.h - driver code for ESP32

  Part of grblHAL

  Copyright (c) 2024 Terje Io

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

#if N_ABC_MOTORS > 0
#error "Axis configuration is not supported!"
#endif

#if VFD_SPINDLE
#error "Generic map does not have support for VFD spindle."
#endif

#if KEYPAD_ENABLE == 1
#error No free pins for I2C keypad!
#endif

#define SERIAL1_PORT // RX: GPIO_NUM_18, TX: GPIO_NUM_17

// Define step pulse output pins.
#define X_STEP_PIN              GPIO_NUM_12
#define Y_STEP_PIN              GPIO_NUM_13
#define Z_STEP_PIN              GPIO_NUM_14

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN         GPIO_NUM_9
#define Y_DIRECTION_PIN         GPIO_NUM_10
#define Z_DIRECTION_PIN         GPIO_NUM_11

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN     GPIO_NUM_8

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN             GPIO_NUM_5
#define Y_LIMIT_PIN             GPIO_NUM_6
#define Z_LIMIT_PIN             GPIO_NUM_7

#define AUXOUTPUT0_PIN          GPIO_NUM_47 // Spindle PWM
#define AUXOUTPUT1_PIN          GPIO_NUM_46 // Spindle enable
#define AUXOUTPUT2_PIN          GPIO_NUM_45 // Spindle direction
#define AUXOUTPUT3_PIN          GPIO_NUM_16 // Coolant flood
#define AUXOUTPUT4_PIN          GPIO_NUM_15 // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PIN           GPIO_NUM_1
#define AUXINPUT1_PIN           GPIO_NUM_2
#if !SDCARD_ENABLE
#define AUXINPUT2_PIN           GPIO_NUM_38 // Reset/EStop
#define AUXINPUT3_PIN           GPIO_NUM_37 // Feed hold
#endif
#define AUXINPUT4_PIN           GPIO_NUM_36 // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if (CONTROL_ENABLE & CONTROL_HALT) && defined(AUXINPUT2_PIN)
#define RESET_PIN               AUXINPUT2_PIN
#endif
#if (CONTROL_ENABLE & CONTROL_FEED_HOLD) && defined(AUXINPUT3_PIN)
#define FEED_HOLD_PIN           AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT4_PIN
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

// Define I2C port/pins
#if I2C_ENABLE
#define I2C_PORT                I2C_NUM_1
#define I2C_SDA                 GPIO_NUM_3
#define I2C_SCL                 GPIO_NUM_4
#define I2C_CLOCK               100000
#endif

#if SDCARD_ENABLE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define SPI_MISO_PIN            GPIO_NUM_37
#define SPI_MOSI_PIN            GPIO_NUM_35
#define SPI_SCK_PIN             GPIO_NUM_36
#define SD_CS_PIN               GPIO_NUM_21
#endif
