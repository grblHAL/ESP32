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

#define BOARD_NAME "BTT Rodent"
#define BOARD_URL "https://github.com/bigtreetech/Rodent/tree/master"
// https://bttwiki.com/Rodent.html

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if KEYPAD_ENABLE == 1
#error No free pins for I2C keypad!
#endif

#if TRINAMIC_ENABLE != 5160
#error BOARD_BTT_RODENT has soldered TMC2160 drivers.
#endif

//#define TRINAMIC_MIXED_DRIVERS 0 Uncomment when board verified

#define USE_I2S_OUT
#define I2S_OUT_PIN_BASE 64

#define I2S_OUT_BCK             GPIO_NUM_22
#define I2S_OUT_WS              GPIO_NUM_17
#define I2S_OUT_DATA            GPIO_NUM_21

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
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN             I2SO(13)
#define M3_DIRECTION_PIN        I2SO(12)
#define M3_ENABLE_PIN           I2SO(15)
#define M3_LIMIT_PIN            GPIO_NUM_32
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PIN             I2SO(18)
#define M4_DIRECTION_PIN        I2SO(17)
#define M4_ENABLE_PIN           I2SO(16)
#define M4_LIMIT_PIN            GPIO_NUM_37
#endif
// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PIN         GPIO_NUM_13
#else
#define AUXOUTPUT1_PIN          GPIO_NUM_13
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PIN   GPIO_NUM_15
#else
#define AUXOUTPUT3_PIN          GPIO_NUM_15
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PIN      GPIO_NUM_25
#else
#define AUXOUTPUT4_PIN          GPIO_NUM_25
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_MIST_PIN        GPIO_NUM_2
#define COOLANT_FLOOD_PIN       GPIO_NUM_4

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.

// N/A

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
#define PIN_NUM_MISO            GPIO_NUM_19
#define PIN_NUM_MOSI            GPIO_NUM_23
#define PIN_NUM_CLK             GPIO_NUM_18
#define MOTOR_CS_PIN            GPIO_NUM_5
#if SDCARD_ENABLE
#define PIN_NUM_CS              GPIO_NUM_2
#endif

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
