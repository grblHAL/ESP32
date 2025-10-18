/*
  fysetc_e4_map.h - driver code for ESP32

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if VFD_SPINDLE
#error "Fystec E4 board does not have support for VFD spindle."
#endif

#if KEYPAD_ENABLE
#error "No free pins for the keypad plugin!"
#endif

#define BOARD_NAME "Fysetc E4 v1.0"

#define SERIAL1_PORT
#define UART1_RX_PIN        GPIO_NUM_21
#define UART1_TX_PIN        GPIO_NUM_22

#if CONTROL_ENABLE & ~CONTROL_HALT
#undef CONTROL_ENABLE
#define CONTROL_ENABLE CONTROL_HALT // Only input supported.
#endif

#if TRINAMIC_ENABLE != 2209
#undef TRINAMIC_ENABLE
#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_ENABLE      2209
#define TRINAMIC_UART_ENABLE 1
#endif

// Define step pulse output pins.
#define X_STEP_PIN          GPIO_NUM_27
#define Y_STEP_PIN          GPIO_NUM_33
#define Z_STEP_PIN          GPIO_NUM_14

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_26
#define Y_DIRECTION_PIN     GPIO_NUM_32
#define Z_DIRECTION_PIN     GPIO_NUM_12

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN GPIO_NUM_25

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN         GPIO_NUM_34
#define Y_LIMIT_PIN         GPIO_NUM_35
#define Z_LIMIT_PIN         GPIO_NUM_15

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         GPIO_NUM_16
#define M3_DIRECTION_PIN    GPIO_NUM_17
//#define M3_LIMIT_PIN        GPIO_NUM_36
#endif

#define AUXOUTPUT0_PIN      GPIO_NUM_13 // Spindle PWM
#define AUXOUTPUT1_PIN      GPIO_NUM_4  // Spindle enable
#define AUXOUTPUT2_PIN      GPIO_NUM_2  // Coolant flood

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN     AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN  AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN   AUXOUTPUT2_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#ifdef COOLANT_FLOOD_PIN
#define COOLANT_ENABLE COOLANT_FLOOD
#else
#define COOLANT_ENABLE 0
#endif
#endif

#define AUXINPUT0_PIN       GPIO_NUM_39 // Probe
#define AUXINPUT1_PIN       GPIO_NUM_36 // Reset/EStop

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN           AUXINPUT1_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define SPI_MISO_PIN        GPIO_NUM_19
#define SPI_MOSI_PIN        GPIO_NUM_23
#define SPI_SCK_PIN         GPIO_NUM_18
#define SD_CS_PIN           GPIO_NUM_5
#endif
