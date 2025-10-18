/*
  bdring_i2s_6x_v3_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

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

#define BOARD_NAME "BDRING 6-axis I2S v3"
#define BOARD_URL "http://wiki.fluidnc.com/en/hardware/official/6_Pack_Integrated_ESP32"

#if KEYPAD_ENABLE == 1
#error No free pins for I2C keypad!
#endif

#include "use_i2s_out.h"

// Module slots

//     1  2  3  4  5  6  7  8
//    -- -- -- -- -- -- -- --
// 1: 33 32 35 34 14 13 15 12
// 2:  2 25 39 36 14 13 15 12
// 3: 26  4 16 27 14 13 15 12
// 4: 14 13 15 12 14 13 15 12

#if SDCARD_ENABLE || TRINAMIC_SPI_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
#define SPI_MISO_PIN            19
#define SPI_MOSI_PIN            23
#define SPI_SCK_PIN             18
#if SDCARD_ENABLE
// Note that a pull-up on CS line is required in SD mode.
#define SD_CS_PIN               5
#endif // SDCARD_ENABLE
#endif // SDCARD_ENABLE || TRINAMIC_SPI_ENABLE

#define I2S_OUT_BCK             GPIO_NUM_22
#define I2S_OUT_WS              GPIO_NUM_17
#define I2S_OUT_DATA            GPIO_NUM_21

#define X_STEP_PIN              I2SO(2)
#define X_DIRECTION_PIN         I2SO(1)
#define X_ENABLE_PIN            I2SO(0)

#define Y_STEP_PIN              I2SO(5)
#define Y_DIRECTION_PIN         I2SO(4)
#define Y_ENABLE_PIN            I2SO(7)

#define Z_STEP_PIN              I2SO(10)
#define Z_DIRECTION_PIN         I2SO(9)
#define Z_ENABLE_PIN            I2SO(8)

#if TRINAMIC_SPI_ENABLE
#define MOTOR_CSX_PIN           I2SO(3)
#define MOTOR_CSY_PIN           I2SO(6)
#define MOTOR_CSZ_PIN           I2SO(11)
#endif

// Slot #1

#define X_LIMIT_PIN             GPIO_NUM_33 // 1
#define Y_LIMIT_PIN             GPIO_NUM_32 // 2
#define Z_LIMIT_PIN             GPIO_NUM_35 // 3

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN             I2SO(13)
#define M3_DIRECTION_PIN        I2SO(12)
#define M3_ENABLE_PIN           I2SO(15)
#define M3_LIMIT_PIN            GPIO_NUM_34 // 4
#if TRINAMIC_SPI_ENABLE
#define MOTOR_CSM3_PIN          I2SO(14)
#endif
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 2
#define M4_AVAILABLE
#define M4_STEP_PIN             I2SO(18)
#define M4_DIRECTION_PIN        I2SO(17)
#define M4_ENABLE_PIN           I2SO(16)
//#define M4_LIMIT_PIN            GPIO_NUM_32
#if TRINAMIC_SPI_ENABLE
#define MOTOR_CSM4_PIN          I2SO(19)
#endif
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PIN             I2SO(21)
#define M5_DIRECTION_PIN        I2SO(20)
#define M5_ENABLE_PIN           I2SO(23)
//#define M5_LIMIT_PIN            GPIO_NUM_33
#if TRINAMIC_SPI_ENABLE
#define MOTOR_CSM5_PIN          I2SO(22)
#endif
#endif

// Slot #3

#define AUXOUTPUT0_PIN          GPIO_NUM_26 // 1 - Spindle PWM
#define AUXOUTPUT1_PIN          GPIO_NUM_4  // 2 - Spindle enable
#define AUXOUTPUT2_PIN          GPIO_NUM_16 // 3 - Spindle direction
#define AUXOUTPUT3_PIN          GPIO_NUM_27 // 4 - Coolant mist

// Define driver spindle pins, on 10V Spindle module
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#undef COOLANT_ENABLE
#ifdef COOLANT_MIST_PIN
#define COOLANT_ENABLE COOLANT_MIST
#else
#define COOLANT_ENABLE 0
#endif
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0 // No control inputs

// Slot #2

#define AUXINPUT0_PIN           GPIO_NUM_2  // 1
#define AUXINPUT1_PIN           GPIO_NUM_25 // 2
#define AUXINPUT2_PIN           GPIO_NUM_39 // 3
#define AUXINPUT3_PIN           GPIO_NUM_36 // 4

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT0_PIN
#endif

// Slot #4

#define AUXOUTPUT4_PIN          GPIO_NUM_12 // 4 or 8

#ifdef ADD_SERIAL1
#define SERIAL1_PORT
#define UART1_RX_PIN            GPIO_NUM_15 // 3
#define UART1_TX_PIN            GPIO_NUM_14 // 1
#if MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED
#define MODBUS_DIRECTION_PIN    GPIO_NUM_13 // 2
#endif
#else
#define AUXOUTPUT5_PIN          GPIO_NUM_14 // 1 or 5
#define AUXOUTPUT6_PIN          GPIO_NUM_13 // 2 or 6
#define AUXOUTPUT7_PIN          GPIO_NUM_15 // 3 or 7
#endif
