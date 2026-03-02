/*
  sourcerabbit_rabbit_board_4-axis.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2021-2022 Terje Io

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

#define BOARD_NAME  "Rabbit Board 4-Axis"
#define BOARD_URL   "https://www.sourcerabbit.com/Shop/pr-i-106-t-rabbit-board-4-axis.htm"

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#error VFD Spindle not supported!
#endif

#if KEYPAD_ENABLE
#error Keypad not supported!
#endif

#if SDCARD_ENABLE
#error SD card not supported!
#endif

// Step & direction pins for X, Y, Z axes
#define X_STEP_PIN              GPIO_NUM_16
#define X_DIRECTION_PIN         GPIO_NUM_33
#define Y_STEP_PIN              GPIO_NUM_25
#define Y_DIRECTION_PIN         GPIO_NUM_26
#define Z_STEP_PIN              GPIO_NUM_27
#define Z_DIRECTION_PIN         GPIO_NUM_14

// Stepper driver enable/disable output pin
#define STEPPERS_ENABLE_PIN     GPIO_NUM_15

// Homing / hard limit switch input pins
#define X_LIMIT_PIN             GPIO_NUM_36
#define Y_LIMIT_PIN             GPIO_NUM_39
#define Z_LIMIT_PIN             GPIO_NUM_34

// A axis (4th axis) step, direction and limit pins
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN             GPIO_NUM_12
#define M3_DIRECTION_PIN        GPIO_NUM_13
#define M3_LIMIT_PIN            GPIO_NUM_35
#endif

// ATC (Automatic Tool Changer) output pins
#define AUXOUTPUT0_PIN          GPIO_NUM_5  // ATC Door opener (solenoid or air valve)
#define AUXOUTPUT1_PIN          GPIO_NUM_18 // ATC Blow
#define AUXOUTPUT2_PIN          GPIO_NUM_19 // ATC Lock

// Spindle output pins
#define AUXOUTPUT3_PIN          GPIO_NUM_2  // Spindle PWM
#define AUXOUTPUT4_PIN          GPIO_NUM_21 // Spindle direction

// Coolant output pins
#define AUXOUTPUT5_PIN          GPIO_NUM_22 // Coolant flood
#define AUXOUTPUT6_PIN          GPIO_NUM_23 // Coolant mist

// Spindle PWM output pin
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT3_PIN
#endif

// Spindle direction output pin
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT4_PIN
#endif

// Coolant output pins
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT5_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT6_PIN
#endif

// No control inputs (cycle start, reset, feed hold)
#undef CONTROL_ENABLE
#define CONTROL_ENABLE          0

// Probe input pin
#define AUXINPUT0_PIN           GPIO_NUM_32

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT0_PIN
#endif