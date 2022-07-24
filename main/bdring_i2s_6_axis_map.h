/*
  bdring_i2s_6_axis_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2020 Terje Io

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
#error "Board BOARD_BDRING_I2S6A does not have support for VFD spindle."
#endif

#define BOARD_NAME "BDRING 6-axis I2S"

#define USE_I2S_OUT

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#endif // SDCARD_ENABLE

#define I2S_OUT_BCK     GPIO_NUM_22
#define I2S_OUT_WS      GPIO_NUM_17
#define I2S_OUT_DATA    GPIO_NUM_21

#define X_STEP_PIN      2
#define X_DIRECTION_PIN 1
#define X_ENABLE_PIN   	0
#define X_LIMIT_PIN     GPIO_NUM_36

#define Y_STEP_PIN      5
#define Y_DIRECTION_PIN 4
#define Y_ENABLE_PIN   	7
#define Y_LIMIT_PIN     GPIO_NUM_39

#define Z_STEP_PIN      10
#define Z_DIRECTION_PIN 9
#define Z_ENABLE_PIN   	8
#define Z_LIMIT_PIN     GPIO_NUM_34

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN      	13
#define M3_DIRECTION_PIN 	12
#define M3_ENABLE_PIN   	15
#define M3_LIMIT_PIN     	GPIO_NUM_35
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 2
#define M4_AVAILABLE
#define M4_STEP_PIN      	18
#define M4_DIRECTION_		PIN 17
#define M4_ENABLE_PIN   	16
#define M4_LIMIT_PIN     	GPIO_NUM_32
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PIN      	21
#define M5_DIRECTION_PIN 	20
#define M5_ENABLE_PIN   	23
#define M5_LIMIT_PIN     	GPIO_NUM_33

#endif

// Define spindle enable and spindle direction output pins.

#define SPINDLEPWMPIN           GPIO_NUM_26
#define SPINDLE_ENABLE_PIN      GPIO_NUM_4
#define SPINDLE_DIRECTION_PIN   GPIO_NUM_16

// Define flood and mist coolant enable output pins.

#define COOLANT_MIST_PIN    GPIO_NUM_26

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.

// N/A

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN   GPIO_NUM_25
#endif

#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif
