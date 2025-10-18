/*
  cnc_boosterpack_s3_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32-S3

  Part of grblHAL

  Copyright (c)2023-2024 Terje Io

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

#define BOARD_NAME "CNC BoosterPack"
#define BOARD_URL "https://github.com/terjeio/CNC_Boosterpack"

#if TRINAMIC_ENABLE
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_MIXED_DRIVERS 0
#ifdef TRINAMIC_I2C
#undef TRINAMIC_I2C
#endif
#define TRINAMIC_I2C 1
#endif

#if !EEPROM_ENABLE
#undef EEPROM_ENABLE
#define EEPROM_ENABLE 1 // I2C EEPROM (24LC16) support.
#endif

// Define step pulse output pins.
#define X_STEP_PIN              GPIO_NUM_26
#define Y_STEP_PIN              GPIO_NUM_27
#define Z_STEP_PIN              GPIO_NUM_14

// Define step direction output pins.
#define X_DIRECTION_PIN         GPIO_NUM_2
#define Y_DIRECTION_PIN         GPIO_NUM_15
#define Z_DIRECTION_PIN         GPIO_NUM_12

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN    IOEXPAND

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN             GPIO_NUM_4
#define Y_LIMIT_PIN             GPIO_NUM_16
#define Z_LIMIT_PIN             GPIO_NUM_32

#define AUXOUTPUT0_PIN          GPIO_NUM_25 // Spindle enable
#define AUXOUTPUT1_PIN          GPIO_NUM_17 // Spindle PWM
#define AUXOUTPUT2_PIN          GPIO_NUM_15 // Spindle direction
#define AUXOUTPUT3_PIN          GPIO_NUM_2  // Coolant flood
#define AUXOUTPUT4_PIN          GPIO_NUM_4  // Coolant mist

// Define driver spindle pins

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         GPIO_NUM_17
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   IOEXPAND
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      IOEXPAND
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN       IOEXPAND
#define COOLANT_MIST_PIN        IOEXPAND

#define AUXINPUT0_PIN           GPIO_NUM_33
#define AUXINPUT1_PIN           GPIO_NUM_34
#define AUXINPUT2_PIN           GPIO_NUM_13 // Probe
#define AUXINPUT3_PIN           GPIO_NUM_35 // Reset/EStop
#define AUXINPUT4_PIN           GPIO_NUM_39 // Feed hold
#define AUXINPUT5_PIN           GPIO_NUM_36 // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN           AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT5_PIN
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT2_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT1_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN          AUXINPUT0_PIN
#endif

#ifdef ADD_SERIAL1
#define SERIAL1_PORT
#define UART1_RX_PIN            GPIO_NUM_33
#define UART1_TX_PIN            GPIO_NUM_25
#if MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED
#define MODBUS_DIRECTION_PIN    GPIO_NUM_25
#endif
#endif

#if MPG_ENABLE == 1
#define UART1_RX_PIN            GPIO_NUM_33
#define MPG_MODE_PIN            GPIO_NUM_25
#endif

// Define I2C port/pins
#define I2C_PORT                I2C_NUM_1
#define I2C_SDA                 GPIO_NUM_21
#define I2C_SCL                 GPIO_NUM_22
#define I2C_CLOCK               100000

#if SDCARD_ENABLE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define SPI_MISO_PIN        GPIO_NUM_19
#define SPI_MOSI_PIN        GPIO_NUM_23
#define SPI_SCK_PIN         GPIO_NUM_18
#define SD_CS_PIN           GPIO_NUM_5
#endif
