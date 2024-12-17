/*
  cnc3040.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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

#if KEYPAD_ENABLE == 1
#error No free pins for I2C keypad!
#endif

#if SDCARD_ENABLE
#error SD card not supported!
#endif

#define BOARD_NAME "CNC3040 4-axis CNC"
#define BOARD_URL "https://github.com/shaise/GrblCNC/tree/master/Hardware/GrblCnc3040"

#ifdef ADD_SERIAL1
#define SERIAL1_PORT // RX: 16, TX: 17
#endif

// Define step pulse output pins.
#define X_STEP_PIN          GPIO_NUM_32
#define Y_STEP_PIN          GPIO_NUM_25
#define Z_STEP_PIN          GPIO_NUM_27

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_33
#define Y_DIRECTION_PIN     GPIO_NUM_26
#define Z_DIRECTION_PIN     GPIO_NUM_18

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN GPIO_NUM_15

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN         GPIO_NUM_36   // VP
#define Y_LIMIT_PIN         GPIO_NUM_39   // VN
#define Z_LIMIT_PIN         GPIO_NUM_34

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         GPIO_NUM_5
#define M3_DIRECTION_PIN    GPIO_NUM_4
#endif

#define AUXOUTPUT0_PIN          GPIO_NUM_2  // Spindle enable
#define AUXOUTPUT1_PIN          GPIO_NUM_21 // Spindle PWM
#define AUXOUTPUT2_PIN          GPIO_NUM_22 // Coolant flood
#define AUXOUTPUT3_PIN          GPIO_NUM_23 // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN     AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PIN  AUXOUTPUT0_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN   AUXOUTPUT2_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN    AUXOUTPUT3_PIN
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0 // No control inputs

#define AUXINPUT0_PIN       GPIO_NUM_35  // ATC Door
#define AUXINPUT1_PIN       GPIO_NUM_25

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT0_PIN  // ATC Door
#endif
