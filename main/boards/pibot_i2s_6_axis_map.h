/*
  pibot_i2s_6_axis_map.h - An embedded CNC Controller

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2025 @luc-github

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

#define BOARD_NAME "PiBot 6-axis I2S"
#define BOARD_URL "https://www.pibot.com/pibot-fluidnc-grbl-cnc-controller-v4-9"

#include "use_i2s_out.h"

#if SDCARD_ENABLE //  || TRINAMIC_SPI_ENABLE 
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
#define SPI_MISO_PIN            19
#define SPI_MOSI_PIN            23
#define SPI_SCK_PIN             18
#if SDCARD_ENABLE
#define SD_CS_PIN               5
#endif // SDCARD_ENABLE
#endif // SDCARD_ENABLE || TRINAMIC_SPI_ENABLE

#define I2S_OUT_BCK             GPIO_NUM_22
#define I2S_OUT_WS              GPIO_NUM_17
#define I2S_OUT_DATA            GPIO_NUM_21

#define X_STEP_PIN              I2SO(2)
#define X_DIRECTION_PIN         I2SO(1)
#define X_ENABLE_PIN            I2SO(0)
#define X_LIMIT_PIN             GPIO_NUM_35
//#define X_CS_PIN                I2SO(3)

#define Y_STEP_PIN              I2SO(5)
#define Y_DIRECTION_PIN         I2SO(4)
#define Y_ENABLE_PIN            I2SO(7)
#define Y_LIMIT_PIN             GPIO_NUM_34
//#define Y_CS_PIN                I2SO(6)


#define Z_STEP_PIN              I2SO(10)
#define Z_DIRECTION_PIN         I2SO(9)
#define Z_ENABLE_PIN            I2SO(8)
#define Z_LIMIT_PIN             GPIO_NUM_39
//#define Z_CS_PIN                I2SO(11)

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN             I2SO(13)
#define M3_DIRECTION_PIN        I2SO(12)
#define M3_ENABLE_PIN           I2SO(15)
#if M3_LIMIT_ENABLE
#define M3_LIMIT_PIN            GPIO_NUM_36
#endif
//#define A_CS_PIN                I2SO(14)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 2
#define M4_AVAILABLE
#define M4_STEP_PIN             I2SO(18)
#define M4_DIRECTION_PIN        I2SO(17)
#define M4_ENABLE_PIN           I2SO(16)
#if M4_LIMIT_ENABLE
#define M4_LIMIT_PIN            GPIO_NUM_32
#endif
//#define B_CS_PIN                I2SO(19)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PIN             I2SO(21)
#define M5_DIRECTION_PIN        I2SO(20)
#define M5_ENABLE_PIN           I2SO(23)
//#define C_CS_PIN                I2SO(22)
#if M5_LIMIT_ENABLE
#define M5_LIMIT_PIN            GPIO_NUM_33
#endif
#endif

#define AUXOUTPUT0_PIN          GPIO_NUM_13 // 10 V
#define AUXOUTPUT1_PIN          I2SO(23)  // Spindle enable
#define AUXOUTPUT2_PIN          GPIO_NUM_12  // Laser
#if !TOOLSETTER_ENABLE
#define AUXOUTPUT3_PIN          GPIO_NUM_26 // Relay or toolsetter input
#endif

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         GPIO_NUM_4
#endif
#if defined(AUXOUTPUT3_PIN) && (DRIVER_SPINDLE_ENABLE & SPINDLE_DIR)
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN       AUXOUTPUT2_PIN
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#undef COOLANT_ENABLE
#ifdef COOLANT_MIST_PIN
#define COOLANT_ENABLE COOLANT_MIST
#else
#define COOLANT_ENABLE 0
#endif
#endif

#define AUXINPUT0_PIN           GPIO_NUM_2
#if !M5_LIMIT_ENABLE
#define AUXINPUT1_PIN           GPIO_NUM_33
#endif
#if !M4_LIMIT_ENABLE
#define AUXINPUT2_PIN           GPIO_NUM_32
#endif
#if !M3_LIMIT_ENABLE
#define AUXINPUT3_PIN           GPIO_NUM_36
#endif
#if TOOLSETTER_ENABLE
#define AUXINPUT4_PIN           GPIO_NUM_26 // Relay or toolsetter input
#define TOOLSETTER_PIN          AUXINPUT4_PIN
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0 // No control inputs

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT0_PIN
#endif

#if I2C_ENABLE
#define I2C_PORT                I2C_NUM_0
#define I2C_SDA                 GPIO_NUM_27
#define I2C_SCL                 GPIO_NUM_25
#define I2C_CLOCK               100000
#endif

#ifdef ADD_SERIAL1
#define SERIAL1_PORT
#define UART1_RX_PIN            GPIO_NUM_15
#define UART1_TX_PIN            GPIO_NUM_16
#if MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED
#define MODBUS_DIRECTION_PIN    GPIO_NUM_14
#endif
#endif
