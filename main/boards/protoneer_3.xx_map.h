/*
  protoneer_3.xx_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

#if N_ABC_MOTORS > 0
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Protoneer v3"

#if VFD_SPINDLE
#error VFD Spindle not supported!
#endif

#if KEYPAD_ENABLE
#error Keypad not supported!
#endif

#if SDCARD_ENABLE
#error SD card not supported!
#endif

// CoolEn not available, use instead E-STOP

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

#define AUXOUTPUT0_PIN      GPIO_NUM_18 // Spindle PWM/spindle direction
#define AUXOUTPUT1_PIN      GPIO_NUM_19 // Spindle enable
#define AUXOUTPUT2_PIN      GPIO_NUM_0  // Coolant flood, NC/BOOT (E-STOP)

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN     AUXOUTPUT0_PIN
#elif DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
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
// N/A
#endif

#define AUXINPUT0_PIN       GPIO_NUM_39 // Probe, A5 (SCL)
#define AUXINPUT1_PIN       GPIO_NUM_36 // Door, A4 (SDA)
#define AUXINPUT2_PIN       GPIO_NUM_2  // Reset/EStop - connected to the "Abort" pin on the shield.
#define AUXINPUT3_PIN       GPIO_NUM_4  // Feed hold
#define AUXINPUT4_PIN       GPIO_NUM_35 // Cycle start - NOTE: an external pullup resistor might be required on this pin.

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN           AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN       AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN     AUXINPUT4_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT0_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT1_PIN
#endif
