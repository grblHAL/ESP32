/*
  espduino-32_wemos_d1_r32_uno_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

/*
  This map is for relatively common ESP32 boards replicating the form factor of Arduino UNO.
  This map allows use of such uno-compatible board with very popular
  "Protoneer Arduino CNC shield" and is based on its pinout.
  This makes perfect match for retrofiting older Arduino+grblHAL based machines
  with 32b microcontroler capable of running grblHAL and providing few extra IO pins (eg. for modbus).

  These boards are sold under several names, for instance:
   + ESPDUINO-32  (USB type B)
   + Wemos D1 R32 (Micro USB)
*/

#if N_ABC_MOTORS > 0
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "ESPDUINO-32 Wemos D1 R32"

// Define step pulse output pins.
#define X_STEP_PIN          GPIO_NUM_26
#define Y_STEP_PIN          GPIO_NUM_25
#define Z_STEP_PIN          GPIO_NUM_17

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_16
#define Y_DIRECTION_PIN     GPIO_NUM_27
#define Z_DIRECTION_PIN     GPIO_NUM_14

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN GPIO_NUM_12

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN         GPIO_NUM_13
#define Y_LIMIT_PIN         GPIO_NUM_5
#define Z_LIMIT_PIN         GPIO_NUM_23

#define AUXOUTPUT0_PIN      GPIO_NUM_19 // Spindle PWM
#define AUXOUTPUT1_PIN      GPIO_NUM_18 // Spindle enable
#if !(MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define AUXOUTPUT2_PIN      GPIO_NUM_32 // Coolant flood
#endif

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN     AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN  AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if (COOLANT_ENABLE & COOLANT_MIST)
#undef COOLANT_ENABLE
#ifdef AUXOUTPUT2_PIN
#define COOLANT_ENABLE COOLANT_FLOOD
#else
#define COOLANT_ENABLE 0
#endif
#endif
#if (COOLANT_ENABLE & COOLANT_FLOOD)
#ifdef AUXOUTPUT2_PIN
#define COOLANT_FLOOD_PIN   AUXOUTPUT2_PIN
#else
#undef COOLANT_ENABLE
#define COOLANT_ENABLE 0
#endif
#endif

#define AUXINPUT0_PIN       GPIO_NUM_39 // Probe
#define AUXINPUT1_PIN       GPIO_NUM_2  // Reset/EStop
#define AUXINPUT2_PIN       GPIO_NUM_4  // Feed hold
#define AUXINPUT3_PIN       GPIO_NUM_35 // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN           AUXINPUT1_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN       AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN     AUXINPUT3_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT0_PIN
#endif

#ifdef ADD_SERIAL1
#define SERIAL1_PORT
#define UART1_RX_PIN            GPIO_NUM_33
#define UART1_TX_PIN            GPIO_NUM_32
#if MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED
#define MODBUS_DIRECTION_PIN    GPIO_NUM_15
#endif
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

#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif
