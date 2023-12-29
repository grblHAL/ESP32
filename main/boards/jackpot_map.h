/*
  jackpot_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2023 Terje Io

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

#define BOARD_NAME "Jackpot"
#define BOARD_URL "https://docs.v1e.com/electronics/jackpot/"

#define USE_I2S_OUT
#define I2S_OUT_PIN_BASE 64

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

#define I2S_OUT_BCK             GPIO_NUM_22
#define I2S_OUT_WS              GPIO_NUM_17
#define I2S_OUT_DATA            GPIO_NUM_21

#define X_STEP_PIN              I2SO(2)
#define X_DIRECTION_PIN         I2SO(1)
#define X_ENABLE_PIN            I2SO(0)
#define X_LIMIT_PIN             GPIO_NUM_25

#define Y_STEP_PIN              I2SO(5)
#define Y_DIRECTION_PIN         I2SO(4)
#define Y_ENABLE_PIN            I2SO(7)
#define Y_LIMIT_PIN             GPIO_NUM_33

#define Z_STEP_PIN              I2SO(10)
#define Z_DIRECTION_PIN         I2SO(9)
#define Z_ENABLE_PIN            I2SO(8)
#define Z_LIMIT_PIN             GPIO_NUM_32

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN             I2SO(13)
#define M3_DIRECTION_PIN        I2SO(12)
#define M3_ENABLE_PIN           I2SO(15)
#define M3_LIMIT_PIN            GPIO_NUM_35
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 2
#define M4_AVAILABLE
#define M4_STEP_PIN             I2SO(18)
#define M4_DIRECTION_PIN        I2SO(17)
#define M4_ENABLE_PIN           I2SO(16)
#define M4_LIMIT_PIN            GPIO_NUM_34
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PIN             I2SO(21)
#define M5_DIRECTION_PIN        I2SO(20)
#define M5_ENABLE_PIN           I2SO(23)
#define M5_LIMIT_PIN            GPIO_NUM_39
#endif

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PIN         GPIO_NUM_26
#else
#define AUXOUTPUT1_PIN          GPIO_NUM_26
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PIN   GPIO_NUM_16
#else
#define AUXOUTPUT3_PIN          GPIO_NUM_16
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PIN      GPIO_NUM_4
#else
#define AUXOUTPUT4_PIN          GPIO_NUM_4
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_MIST_PIN    GPIO_NUM_16
#define COOLANT_FLOOD_PIN   GPIO_NUM_2

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.

// N/A

#define AUXOUTPUT0_PIN      GPIO_NUM_26
#define AUXOUTPUT1_PIN      GPIO_NUM_27

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO        GPIO_NUM_19
#define PIN_NUM_MOSI        GPIO_NUM_23
#define PIN_NUM_CLK         GPIO_NUM_18
#define MOTOR_CS_PIN        GPIO_NUM_17
#if SDCARD_ENABLE
#define PIN_NUM_CS          GPIO_NUM_5
#endif

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#define UART2_RX_PIN            GPIO_NUM_4
#define UART2_TX_PIN            GPIO_NUM_0
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN               GPIO_NUM_36
#endif

#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif
