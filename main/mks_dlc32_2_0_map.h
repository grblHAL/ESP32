/*
  mks_dlc32_2_0_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2023 Terje Io
  Copyright (c) 2022 Lucio Tarantino

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

#if VFD_SPINDLE
#error "Board BOARD_MKS_DLC32_V2P0 does not have support for VFD spindle."
#endif

#define BOARD_NAME "MKS DLC32 2.0"

#define USE_I2S_OUT
#define I2S_OUT_PIN_BASE 64

// timer definitions
#define STEP_TIMER_GROUP    TIMER_GROUP_0
#define STEP_TIMER_INDEX    TIMER_0

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO        GPIO_NUM_12
#define PIN_NUM_MOSI        GPIO_NUM_13
#define PIN_NUM_CLK         GPIO_NUM_14
#define PIN_NUM_CS          GPIO_NUM_15

#endif // SDCARD_ENABLE

#define I2S_OUT_BCK         GPIO_NUM_16
#define I2S_OUT_WS          GPIO_NUM_17
#define I2S_OUT_DATA        GPIO_NUM_21

#define X_STEP_PIN          I2SO(1)
#define X_DIRECTION_PIN     I2SO(2)
#define X_LIMIT_PIN         GPIO_NUM_36

#define Y_STEP_PIN          I2SO(5)
#define Y_DIRECTION_PIN     I2SO(6)
#define Y_LIMIT_PIN         GPIO_NUM_35

#define Z_STEP_PIN          I2SO(3)
#define Z_DIRECTION_PIN     I2SO(4)
#define Z_LIMIT_PIN         GPIO_NUM_34

#define STEPPERS_ENABLE_PIN I2SO(0)

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 1
#error "Board BOARD_MKS_DLC32_V2P0 does not have support for ABC Motors"
#endif

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PIN     GPIO_NUM_32
#else
#define AUXOUTPUT4_PIN      GPIO_NUM_32
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PIN  GPIO_NUM_27
#else
#define AUXOUTPUT5_PIN      GPIO_NUM_27
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_MIST_PIN    I2SO(7)
#define COOLANT_FLOOD_PIN   GPIO_NUM_5

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.

#if !I2C_ENABLE
#define FEED_HOLD_PIN       GPIO_NUM_0  // J2,3 (I2C_SDA)
#define CYCLE_START_PIN     GPIO_NUM_4  // J2,4 (I2C_SCL)
#endif

#define AUXINPUT0_PIN       GPIO_NUM_33 // EXP_1,8 (LCD_RS)
#define AUXOUTPUT0_PIN      GPIO_NUM_25 // EXP_1,7 (LCD_CS_0)
#define AUXOUTPUT1_PIN      GPIO_NUM_26 // EXP_1,5 (LCD_TOUCH_CS_0)
#define AUXOUTPUT2_PIN      GPIO_NUM_27 // EXP_1,4 (LCD_RST_0)
#define AUXOUTPUT3_PIN      GPIO_NUM_5  // EXP_1,3 (LCD_EN_0)

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN           GPIO_NUM_22
#endif

#if I2C_ENABLE
// Define I2C port/pins
#define I2C_SDA             GPIO_NUM_0
#define I2C_SCL             GPIO_NUM_4
#define I2C_CLOCK           100000
#endif
