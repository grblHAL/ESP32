/*
  sourcerabbit_4axis.h - An embedded CNC Controller with rs274/ngc (g-code) support

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

#ifdef BOARD_SOURCERABBIT_4AXIS_12
#define BOARD_NAME "SourceRabbit 4-axis CNC v1.2"
#else
#define BOARD_NAME "SourceRabbit 4-axis CNC"
#endif
#define BOARD_URL "https://www.sourcerabbit.com/Shop/pr-i-86-t-4-axis-cnc-motherboard.htm"

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#error VFD Spindle not supported!
#endif

#if KEYPAD_ENABLE
#error Keypad not supported!
#endif

#if SDCARD_ENABLE
#error SD card not supported!
#endif

// Define step pulse output pins.
#define X_STEP_PIN          GPIO_NUM_0
#define Y_STEP_PIN          GPIO_NUM_25
#define Z_STEP_PIN          GPIO_NUM_27

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_33
#define Y_DIRECTION_PIN     GPIO_NUM_26
#define Z_DIRECTION_PIN     GPIO_NUM_14

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN GPIO_NUM_15

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN         GPIO_NUM_36
#define Y_LIMIT_PIN         GPIO_NUM_39
#define Z_LIMIT_PIN         GPIO_NUM_34

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         GPIO_NUM_12
#define M3_DIRECTION_PIN    GPIO_NUM_13
#endif

#ifdef BOARD_SOURCERABBIT_4AXIS_12

#define AUXINPUT1_PIN       GPIO_NUM_5  // ATC Door
#define AUXOUTPUT0_PIN      GPIO_NUM_19 // ATC Lock
#define AUXOUTPUT1_PIN      GPIO_NUM_18 // ATC Blow
#define AUXOUTPUT2_PIN      GPIO_NUM_21 // Spindle PWM
#define AUXOUTPUT3_PIN      GPIO_NUM_2  // Spindle direction
#define AUXOUTPUT4_PIN      GPIO_NUM_22 // Coolant flood
#define AUXOUTPUT5_PIN      GPIO_NUM_23 // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT4_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT5_PIN
#endif

#else

#define AUXOUTPUT0_PIN      GPIO_NUM_21 // Spindle PWM
#define AUXOUTPUT1_PIN      GPIO_NUM_2  // Spindle enable
#define AUXOUTPUT3_PIN      GPIO_NUM_22 // Coolant flood
#define AUXOUTPUT4_PIN      GPIO_NUM_23 // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
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

#endif // BOARD_SOURCERABBIT_4AXIS_12

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0 // No control inputs

#define AUXINPUT0_PIN       GPIO_NUM_32

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT0_PIN
#endif

#if SAFETY_DOOR_ENABLE && defined(AUXINPUT1_PIN)
#define SAFETY_DOOR_PIN     AUXINPUT1_PIN
#endif
