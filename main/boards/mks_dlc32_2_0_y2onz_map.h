/*
  mks_dlc32_2_0_y2onz_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for DLC32 v2.1 (ESP32). Using Y2 motor on Z axis driver. No Z axis support.

  Part of grblHAL

  Copyright (c) 2023-2024 Terje Io
  Copyright (c) 2022 Lucio Tarantino
  Copyright (c) 2025 Kari Brown

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

#if VFD_SPINDLE
#error "Board BOARD_MKS_DLC32_V2P0 does not have support for VFD spindle."
#endif

#include "use_i2s_out.h"

#define BOARD_MKS_DLC32_V2P0_Y2ONZ 1
#undef BOARD_NAME
#define BOARD_NAME "MKS DLC32 2.x (Y2 on Z driver)"
#define BOARD_URL "https://github.com/makerbase-mks/MKS-DLC32"

#define AUX_CONTROLS_OUT

#ifdef ADD_SERIAL1
#define SERIAL1_PORT
#define UART1_RX_PIN GPIO_NUM_18
#define UART1_TX_PIN GPIO_NUM_19
#endif

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define SPI_MISO_PIN GPIO_NUM_12
#define SPI_MOSI_PIN GPIO_NUM_13
#define SPI_SCK_PIN GPIO_NUM_14
#define SD_CS_PIN GPIO_NUM_15
#define SD_DETECT_PIN GPIO_NUM_39

#endif // SDCARD_ENABLE

#define I2S_OUT_BCK GPIO_NUM_16
#define I2S_OUT_WS GPIO_NUM_17
#define I2S_OUT_DATA GPIO_NUM_21

#define X_STEP_PIN I2SO(1)
#define X_DIRECTION_PIN I2SO(2)
#define X_LIMIT_PIN GPIO_NUM_36

#define Y_STEP_PIN I2SO(5)
#define Y_DIRECTION_PIN I2SO(6)
#define Y_LIMIT_PIN GPIO_NUM_35

// Original Z mapping disabled
// #define Z_STEP_PIN          I2SO(3)
// #define Z_DIRECTION_PIN     I2SO(4)
// #define Z_LIMIT_PIN         GPIO_NUM_34

#define STEPPERS_ENABLE_PIN I2SO(0)

///////////////////////////////////////////////////////////////////
// Modification to use Z as ganged Y2
///////////////////////////////////////////////////////////////////
#define N_AXIS 3
#undef Y_GANGED
#define Y_GANGED 1
// Tell the core not to invent M3-based Y2 mappings
#undef M3_STEP_PIN
#undef M3_DIRECTION_PIN
#undef M3_LIMIT_PIN
#undef M3_ENABLE_PIN
// Repurposed former Z channel → Y2 for auto-square
// Make ganged-Y use motor slot M3 (what motor_pins.h expects)
#ifndef M3_STEP_PIN
#define M3_STEP_PIN I2SO(3) // use your current Y2 STEP bit
#endif                      // M3_STEP_PIN
#ifndef M3_DIRECTION_PIN
#define M3_DIRECTION_PIN I2SO(4) // confirmed correct by your Z test
#endif                           // M3_DIRECTION_PIN
// #ifndef M3_LIMIT_PIN_MIN
// #define M3_LIMIT_PIN_MIN GPIO_NUM_34 // reuse old Z limit pin
// #endif
#ifndef Y2_LIMIT_PIN // Y2_LIMIT_PIN
#define Y2_LIMIT_PIN GPIO_NUM_34
#endif // Y2_LIMIT_PIN

#ifndef Y_AUTO_SQUARE
#define Y_AUTO_SQUARE 1
#endif
#define Y_GANGED_LIM_MIN 1

// Z axis disabled (no driver connected)
#define Z_STEP_PIN GPIO_NUM_NC
#define Z_DIRECTION_PIN GPIO_NUM_NC
#undef Z_LIMIT_PIN

///////////////////////////////////////////////////////////////////

// Define ganged axis or A axis step pulse and step direction output pins.
// This guard is disabled since we're repusposing Z as Y2.
// #if N_ABC_MOTORS >= 1
// #error "Board BOARD_MKS_DLC32_V2P0 does not have support for ABC Motors"
// #endif

#define AUXOUTPUT0_PIN GPIO_NUM_25 // EXP_1,7 (LCD_CS_0)
#if PWM_SERVO_ENABLE || (SPINDLE_ENABLE & ((1 << SPINDLE_PWM2) | (1 << SPINDLE_PWM2_NODIR)))
#define AUXOUTPUT0_PWM_PIN GPIO_NUM_26 // EXP_1,5 (LCD_TOUCH_CS_0)
#else
#define AUXOUTPUT1_PIN GPIO_NUM_26 // EXP_1,5 (LCD_TOUCH_CS_0)
#endif
#define AUXOUTPUT2_PIN GPIO_NUM_27 // EXP_1,4 (LCD_RST_0)
#define AUXOUTPUT3_PIN GPIO_NUM_32 // LC
#define AUXOUTPUT4_PIN GPIO_NUM_5  //  EXP_1,3 (LCD_EN_0)

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & (SPINDLE_ENA | SPINDLE_PWM)
#define SPINDLE_PWM_PIN AUXOUTPUT3_PIN
#define SPINDLE_ENABLE_PIN AUXOUTPUT2_PIN
#elif DRIVER_SPINDLE_ENABLE & (SPINDLE_ENA | SPINDLE_DIR)
#define SPINDLE_ENABLE_PIN AUXOUTPUT3_PIN
#define SPINDLE_DIRECTION_PIN AUXOUTPUT2_PIN
#elif DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN AUXOUTPUT3_PIN
#endif

#if DRIVER_SPINDLE1_ENABLE & (SPINDLE_ENA | SPINDLE_PWM)
#define SPINDLE1_PWM_PIN AUXOUTPUT0_PIN
#define SPINDLE1_ENABLE_PIN AUXOUTPUT1_PIN
#elif DRIVER_SPINDLE1_ENABLE & (SPINDLE_ENA | SPINDLE_DIR)
#define SPINDLE1_ENABLE_PIN AUXOUTPUT1_PIN
#define SPINDLE1_DIRECTION_PIN AUXOUTPUT0_PIN
#elif DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA
#define SPINDLE1_ENABLE_PIN AUXOUTPUT1_PIN
#endif

#define SPINDLE_DIRECTION_PIN GPIO_NUM_NC

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN AUXOUTPUT4_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN I2SO(7) // Beeper
#endif

#define AUXINPUT0_PIN GPIO_NUM_33 // EXP_1,8 (LCD_RS)
#define AUXINPUT1_PIN GPIO_NUM_22
#if !I2C_ENABLE
#define AUXINPUT2_PIN GPIO_NUM_4 // Cycle start  // J2,4 (I2C_SCL)
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if (CONTROL_ENABLE & CONTROL_CYCLE_START) && defined(AUXINPUT2_PIN)
#define CYCLE_START_PIN AUXINPUT2_PIN
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
// #define PROBE_PIN AUXINPUT1_PIN
#define PROBE_PIN GPIO_NUM_33
#endif

#define STR1(x) #x
#define STR(x) STR1(x)

// #if PROBE_ENABLE
// #ifdef PROBE_PIN
// #pragma message("[BUILD CHECK] PROBE_ENABLE=1, PROBE_PIN=" STR(PROBE_PIN))
// #else
// #pragma message("[BUILD CHECK] PROBE_ENABLE=1, but PROBE_PIN is UNDEFINED")
// #endif
// #else
// #pragma message("[BUILD CHECK] PROBE_ENABLE=0")
// #endif

#if I2C_ENABLE
// Define I2C port/pins
#define I2C_PORT I2C_NUM_1
#define I2C_SDA GPIO_NUM_0
#define I2C_SCL GPIO_NUM_4
#define I2C_CLOCK 100000
#endif

#if MPG_ENABLE == 1
// Use GPIO33 (EXP1) for MPG_MODE.
// MPG UART is IO18/19 (EXP2).
#define MPG_MODE_PIN AUXINPUT0_PIN
#endif
