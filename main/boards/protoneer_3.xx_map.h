/*
  protoneer_3.xx_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

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

#if N_ABC_MOTORS > 0
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Protoneer v3"

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

#if VFD_SPINDLE
#error VFD Spindle not supported!
#endif

#if KEYPAD_ENABLE
#error Keypad not supported!
#endif

#if SDCARD_ENABLE
#error SD card not supported!
#endif

// Define step pulse output pins.
#define X_STEP_PIN          GPIO_NUM_26 // D2
#define Y_STEP_PIN          GPIO_NUM_25 // D3
#define Z_STEP_PIN          GPIO_NUM_17 // D4

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_16 // D5
#define Y_DIRECTION_PIN     GPIO_NUM_27 // D6
#define Z_DIRECTION_PIN     GPIO_NUM_14 // D7

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN GPIO_NUM_12 // D8

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN         GPIO_NUM_13 // D9
#define Y_LIMIT_PIN         GPIO_NUM_5  // D10
#define Z_LIMIT_PIN         GPIO_NUM_23 // D11

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE // D12
#define SPINDLE_PWM_PIN         GPIO_NUM_19
#else
#define AUXOUTPUT0_PIN          GPIO_NUM_19
#endif

#if DRIVER_SPINDLE_ENABLE // D13
#define SPINDLE_ENABLE_PIN      GPIO_NUM_18
#else
#define AUXOUTPUT1_PIN          GPIO_NUM_18
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_34 // A3
#define COOLANT_MIST_PIN    GPIO_NUM_36 // A4

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           GPIO_NUM_2 // A0
#define FEED_HOLD_PIN       GPIO_NUM_4 // A1
#define CYCLE_START_PIN     GPIO_NUM_35 // A2

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN           GPIO_NUM_39 // A5
#endif
