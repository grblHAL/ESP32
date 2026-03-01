/*
  rabbit_board_4axis_map.h - Board map for the SourceRabbit Rabbit Board 4-Axis CNC

  Compatible with: grblHAL ESP32 driver
  Processor: ESP32

  Copyright (c) 2024 - grblHAL compatible board map

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#pragma once

// ─────────────────────────────────────────────
// Compatibility checks: only 1 extra axis (A-axis) is allowed
// ─────────────────────────────────────────────
#if N_ABC_MOTORS > 1
#error "Rabbit Board 4-Axis: Only one extra axis (A) is supported!"
#endif

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#error "Rabbit Board 4-Axis: VFD Spindle via Modbus is not supported!"
#endif

#if KEYPAD_ENABLE
#error "Rabbit Board 4-Axis: Keypad is not supported!"
#endif

#if SDCARD_ENABLE
#error "Rabbit Board 4-Axis: SD Card is not supported!"
#endif

// ─────────────────────────────────────────────
// Board name and URL
// ─────────────────────────────────────────────
#define BOARD_NAME "Rabbit Board 4-Axis"
#define BOARD_URL  "https://www.sourcerabbit.com/Shop/pr-i-106-t-rabbit-board-4-axis.htm"

// ─────────────────────────────────────────────
// Stepper defaults (grblHAL settings.c)
// ─────────────────────────────────────────────

// $0 - Step pulse duration (microseconds)
#define DEFAULT_STEP_PULSE_MICROSECONDS     10

// $1 - Stepper idle lock time (255 = keep motors always enabled)
#define DEFAULT_STEPPER_IDLE_LOCK_TIME      255

// $3 - Direction invert mask (bit mask: bit0=X, bit1=Y, bit2=Z)
// Value 4 = invert Z only (bit2). NOTE: In Rabbit GRBL this was "Invert X and Y" (value 3).
// Change to 3 if you need to invert X+Y instead of Z.
#define DEFAULT_DIRECTION_INVERT_MASK       4

// $5 - Invert limit switch signals (0 = normal/no invert)
#define DEFAULT_INVERT_LIMIT_PINS           0

// $6 - Invert probe pin signal (0 = normal/no invert)
#define DEFAULT_INVERT_PROBE_PIN            0

// $11 - Junction deviation (mm)
#define DEFAULT_JUNCTION_DEVIATION          0.025f

// $12 - Arc tolerance (mm)
#define DEFAULT_ARC_TOLERANCE               0.005f

// ─────────────────────────────────────────────
// Soft/Hard Limits & Homing
// ─────────────────────────────────────────────

// Soft limits disabled
#define DEFAULT_SOFT_LIMIT_ENABLE           0

// Hard limits disabled
#define DEFAULT_HARD_LIMIT_ENABLE           0

// Homing disabled
#define DEFAULT_HOMING_ENABLE               0

// $23 - Homing direction mask: Z positive, X/Y negative
#define DEFAULT_HOMING_DIR_MASK             3

// $24 - Homing feed rate (mm/min)
#define DEFAULT_HOMING_FEED_RATE            500.0f

// $25 - Homing seek rate (mm/min)
#define DEFAULT_HOMING_SEEK_RATE            1500.0f

// $27 - Homing pull-off distance (mm)
#define DEFAULT_HOMING_PULLOFF              2.0f

// ─────────────────────────────────────────────
// Maximum axis travel distances
// ─────────────────────────────────────────────

// $130 - X axis maximum travel (mm)
#define DEFAULT_X_MAX_TRAVEL                250.0f

// $131 - Y axis maximum travel (mm)
#define DEFAULT_Y_MAX_TRAVEL                250.0f

// $132 - Z axis maximum travel (mm)
#define DEFAULT_Z_MAX_TRAVEL                60.0f

// $133 - A axis maximum travel (0 = soft limits disabled for A-axis)
#define DEFAULT_A_MAX_TRAVEL                0.0f

// ─────────────────────────────────────────────
// Steps, rates and accelerations per axis
// ─────────────────────────────────────────────

// X Axis
#define DEFAULT_X_STEPS_PER_MM              400.0f  // $100 steps/mm
#define DEFAULT_X_MAX_RATE                  4000.0f // $110 mm/min
#define DEFAULT_X_ACCELERATION              60.0f   // $120 mm/sec^2

// Y Axis
#define DEFAULT_Y_STEPS_PER_MM              400.0f  // $101 steps/mm
#define DEFAULT_Y_MAX_RATE                  4000.0f // $111 mm/min
#define DEFAULT_Y_ACCELERATION              60.0f   // $121 mm/sec^2

// Z Axis
#define DEFAULT_Z_STEPS_PER_MM              400.0f  // $102 steps/mm
#define DEFAULT_Z_MAX_RATE                  2500.0f // $112 mm/min
#define DEFAULT_Z_ACCELERATION              60.0f   // $122 mm/sec^2

// A Axis (rotary or 4th linear axis)
#define DEFAULT_A_STEPS_PER_MM              26.666f // $103 steps/mm or steps/degree
#define DEFAULT_A_MAX_RATE                  7200.0f // $113 mm/min or degrees/min
#define DEFAULT_A_ACCELERATION              60.0f   // $123 mm/sec^2

// ─────────────────────────────────────────────
// Spindle defaults
// ─────────────────────────────────────────────

// Minimum and maximum spindle RPM
#define DEFAULT_SPINDLE_RPM_MIN             0.0f     // RPM
#define DEFAULT_SPINDLE_RPM_MAX             25000.0f // RPM

// ─────────────────────────────────────────────
// Motor pins - Step & Direction
// ─────────────────────────────────────────────

// X Axis
#define X_STEP_PIN                          GPIO_NUM_16
#define X_DIRECTION_PIN                     GPIO_NUM_33

// Y Axis
#define Y_STEP_PIN                          GPIO_NUM_25
#define Y_DIRECTION_PIN                     GPIO_NUM_26

// Z Axis
#define Z_STEP_PIN                          GPIO_NUM_27
#define Z_DIRECTION_PIN                     GPIO_NUM_14

// Enable/disable all stepper drivers
// NOTE: In Rabbit GRBL this was called STEPPERS_DISABLE_PIN.
// In grblHAL the macro is STEPPERS_ENABLE_PIN (same pin, different name).
#define STEPPERS_ENABLE_PIN                 GPIO_NUM_15

// ─────────────────────────────────────────────
// A-Axis (4th motor - referred to as M3 in grblHAL)
// NOTE: In Rabbit GRBL these were A_STEP_PIN / A_DIRECTION_PIN.
// In grblHAL the 4th axis is declared as M3.
// ─────────────────────────────────────────────
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN                         GPIO_NUM_12
#define M3_DIRECTION_PIN                    GPIO_NUM_13
#define M3_LIMIT_PIN                        GPIO_NUM_35
#endif

// ─────────────────────────────────────────────
// Limit switches (Endstops)
// GPIO 34, 35, 36, 39 are input-only pins (correct choice for endstops)
// ─────────────────────────────────────────────
#define X_LIMIT_PIN                         GPIO_NUM_36
#define Y_LIMIT_PIN                         GPIO_NUM_39
#define Z_LIMIT_PIN                         GPIO_NUM_34
// A_LIMIT_PIN (GPIO_NUM_35) is defined above as M3_LIMIT_PIN

// ─────────────────────────────────────────────
// Auxiliary outputs (AUXOUTPUT)
// grblHAL uses AUXOUTPUTx as intermediate macros
// for spindle, coolant, ATC, etc.
// ─────────────────────────────────────────────

// Spindle outputs
#define AUXOUTPUT0_PIN                      GPIO_NUM_2  // Spindle PWM
#define AUXOUTPUT1_PIN                      GPIO_NUM_21 // Spindle Direction

// Coolant outputs
#define AUXOUTPUT2_PIN                      GPIO_NUM_22 // Coolant Mist
#define AUXOUTPUT3_PIN                      GPIO_NUM_23 // Coolant Flood

// ATC - Automatic Tool Changer outputs
// NOTE: In Rabbit GRBL these were USER_DIGITAL_PIN_0 and USER_DIGITAL_PIN_1.
// In grblHAL we use AUXOUTPUT4 and AUXOUTPUT5.
#define AUXOUTPUT4_PIN                      GPIO_NUM_19 // ATC Lock
#define AUXOUTPUT5_PIN                      GPIO_NUM_18 // ATC Blow

// ─────────────────────────────────────────────
// Spindle pin assignment via DRIVER_SPINDLE_ENABLE
// NOTE: Rabbit GRBL used SPINDLE_TYPE and ESpindleType::PWM.
// grblHAL uses conditional defines with DRIVER_SPINDLE_ENABLE instead.
// ─────────────────────────────────────────────
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN                     AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN               AUXOUTPUT1_PIN
#endif

// ─────────────────────────────────────────────
// Coolant pin assignment
// ─────────────────────────────────────────────
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN                    AUXOUTPUT2_PIN
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN                   AUXOUTPUT3_PIN
#endif

// ─────────────────────────────────────────────
// Auxiliary inputs (AUXINPUT)
// ─────────────────────────────────────────────

// Probe input (GPIO_NUM_32 - general purpose input)
#define AUXINPUT0_PIN                       GPIO_NUM_32 // Probe

// ATC Door sensor input
// NOTE: In Rabbit GRBL this was USER_DIGITAL_PIN_2.
// In grblHAL we use AUXINPUT1.
#define AUXINPUT1_PIN                       GPIO_NUM_5  // ATC Door

// ─────────────────────────────────────────────
// Probe pin assignment
// ─────────────────────────────────────────────
#if PROBE_ENABLE
#define PROBE_PIN                           AUXINPUT0_PIN
#endif

// ─────────────────────────────────────────────
// Safety door (uses the ATC Door input)
// ─────────────────────────────────────────────
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN                     AUXINPUT1_PIN
#endif

// ─────────────────────────────────────────────
// Control inputs: disabled (no available pins)
// ─────────────────────────────────────────────
#undef CONTROL_ENABLE
#define CONTROL_ENABLE                      0