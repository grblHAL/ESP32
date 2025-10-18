/*
  btt_rodent_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2024 Terje Io

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

// Force four motors in order to keep the TMC driver SPI chain intact
#if N_ABC_MOTORS == 0 && N_GANGED == 0
#undef N_ABC_MOTORS
#undef N_GANGED
#undef Y_GANGED
#define N_ABC_MOTORS 1
#define N_GANGED 1
#define Y_GANGED 1
#endif

#if KEYPAD_ENABLE == 1
#error No free pins for I2C keypad!
#endif

#if TRINAMIC_ENABLE != 5160
//#error BOARD_BTT_RODENT has soldered TMC2160 drivers.
#endif

#include "use_i2s_out.h"

#define BOARD_NAME "BTT Rodent"
#define BOARD_URL "https://github.com/bigtreetech/Rodent/tree/master"
// https://bttwiki.com/Rodent.html


#define I2S_OUT_BCK             GPIO_NUM_22
#define I2S_OUT_WS              GPIO_NUM_17
#define I2S_OUT_DATA            GPIO_NUM_21

// General TMC settings 
#define TMC_STEALTHCHOP         0 // Disable stealthchop


#define X_STEP_PIN              I2SO(2)
#define X_DIRECTION_PIN         I2SO(1)
#define X_ENABLE_PIN            I2SO(0)
#define X_LIMIT_PIN             GPIO_NUM_35

#define Y_STEP_PIN              I2SO(5)
#define Y_DIRECTION_PIN         I2SO(4)
#define Y_ENABLE_PIN            I2SO(7)
#define Y_LIMIT_PIN             GPIO_NUM_34


#define Z_STEP_PIN              I2SO(10)
#define Z_DIRECTION_PIN         I2SO(9)
#define Z_ENABLE_PIN            I2SO(8)
#define Z_LIMIT_PIN             GPIO_NUM_33

// Define ganged axis or A axis step pulse and step direction output pins.
#define M3_AVAILABLE
#define M3_STEP_PIN             I2SO(13)
#define M3_DIRECTION_PIN        I2SO(12)
#define M3_ENABLE_PIN           I2SO(15)
#define M3_LIMIT_PIN            GPIO_NUM_32

#define AUXOUTPUT0_PIN          GPIO_NUM_25 // Spindle enable
#define AUXOUTPUT1_PIN          GPIO_NUM_13 // Spindle PWM
#define AUXOUTPUT2_PIN          GPIO_NUM_15 // Spindle direction
#define AUXOUTPUT3_PIN          GPIO_NUM_2  // Coolant flood
#define AUXOUTPUT4_PIN          GPIO_NUM_4  // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT0_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0 // No control inputs

#define AUXINPUT0_PIN           GPIO_NUM_36

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT0_PIN
#endif

// Define I2C port/pins
#define I2C_PORT                I2C_NUM_1
#define I2C_SDA                 GPIO_NUM_27
#define I2C_SCL                 GPIO_NUM_26
#define I2C_CLOCK               100000

// Define SPI port/pins
#define SPI_MISO_PIN            GPIO_NUM_19
#define SPI_MOSI_PIN            GPIO_NUM_23
#define SPI_SCK_PIN             GPIO_NUM_18
#define MOTOR_CS_PIN            GPIO_NUM_5
#if SDCARD_ENABLE
#define SD_CS_PIN               GPIO_NUM_0
#endif

//#define TMC_ONE_SPI

// Define Modbus serial port pins
#if MODBUS_ENABLE
#if !(MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED)
#undef MODBUS_ENABLE
#define MODBUS_ENABLE (MODBUS_RTU_ENABLED|MODBUS_RTU_DIR_ENABLED)
#endif
#define SERIAL1_PORT
#define UART1_RX_PIN            GPIO_NUM_16
#define UART1_TX_PIN            GPIO_NUM_15
#define MODBUS_DIRECTION_PIN    GPIO_NUM_14
#endif
