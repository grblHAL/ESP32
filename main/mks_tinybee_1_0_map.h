/*
  mks_tinybee_1_0_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2022 Ennio Sesana

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

#define BOARD_NAME "MKS Tinybee V1.0"

#define USE_I2S_OUT
#define I2S_OUT_PIN_BASE 64
#define I2S_SPINDLE
#define I2S_COOLANT

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

#define I2S_OUT_BCK     GPIO_NUM_25
#define I2S_OUT_WS      GPIO_NUM_26
#define I2S_OUT_DATA    GPIO_NUM_27

#define X_STEP_PIN      	I2SO(1) 
#define X_DIRECTION_PIN 	I2SO(2) 
#define X_ENABLE_PIN   		I2SO(0) 
#define X_LIMIT_PIN     GPIO_NUM_33

#define Y_STEP_PIN      	I2SO(4)
#define Y_DIRECTION_PIN 	I2SO(5)
#define Y_ENABLE_PIN   		I2SO(3)
#define Y_LIMIT_PIN     GPIO_NUM_32

#define Z_STEP_PIN      	I2SO(7)
#define Z_DIRECTION_PIN 	I2SO(8)
#define Z_ENABLE_PIN   		I2SO(6)
#define Z_LIMIT_PIN     GPIO_NUM_22

#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN      	I2SO(10)
#define M3_DIRECTION_PIN 	I2SO(11)
#define M3_ENABLE_PIN   	I2SO(9)
#define M3_LIMIT_PIN     	GPIO_NUM_19
#endif

#if N_ABC_MOTORS >= 2
#define M4_AVAILABLE
#define M4_STEP_PIN      	I2SO(13)
#define M4_DIRECTION_PIN	I2SO(14)
#define M4_ENABLE_PIN   	I2SO(12)
#define M4_LIMIT_PIN     	GPIO_NUM_18
#endif

// Define spindle enable and spindle direction output pins.

#define SPINDLEPWMPIN           GPIO_NUM_2
#define SPINDLE_ENABLE_PIN      I2SO(17)
#define SPINDLE_DIRECTION_PIN   I2SO(18)

//#if MODBUS_ENABLE
//#define UART2_RX_PIN        GPIO_NUM_16
//#define UART2_TX_PIN        GPIO_NUM_17
//#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN       I2SO(19)
#define COOLANT_MIST_PIN        I2SO(20)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define CYCLE_START_PIN         GPIO_NUM_36
#define FEED_HOLD_PIN           GPIO_NUM_34
//#define RESET_PIN             (use board hardware)
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         GPIO_NUM_39
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN   GPIO_NUM_35
#endif

/*
  CONNECTION
  
  X_AXIS - X_AXIS
  Y_AXIS - Y_AXIS
  Z_AXIS - Z_AXIS
  E0 - A_AXIS
  E1 - B_AXIS

  X_LIM - X_LIM
  Y_LIM - Y_LIM
  Z_LIM - Z_LIM
  EXP2 (IO19) - A_LIM
  EXP2 (IO18) - B_LIM
  MT_DET - PROBE
  
  TH1 - CYCLE START
  TH2 - FEED HOLD
  TB - SAFETY DOOR

  3DTOUCH - SPINDLE PWM (NO USE CENTRAL PIN (+5V))
  HE0 - SPINDLE ENABLE
  HE1 - SPINDLE DIRECTION
  FAN1 - FLOOD
  FAN2 - MIST
------------------------------------------------------------------------------------
  IMPORTANT
  change in driver.c
  
  DIGITAL_OUT/DIGITAL_IN instead of gpio_set_level/gpio_get_level for
  
  SPINDLE_ENABLE_PIN
  SPINDLE_DIRECTION_PIN
  COOLANT_FLOOD_PIN
  COOLANT_MIST_PIN

e.g.
    before 
        gpio_set_level(COOLANT_FLOOD_PIN, mode.flood ? 1 : 0);
    after
      DIGITAL_OUT(COOLANT_FLOOD_PIN, mode.flood ? 1 : 0);
    
    before 
        state.on = gpio_get_level(SPINDLE_ENABLE_PIN) != 0;
    after
      state.on = DIGITAL_IN(SPINDLE_ENABLE_PIN) != 0;

Thank's to Terje Io
*/
