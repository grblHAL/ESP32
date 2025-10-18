/*
  mks_tinybee_1_0_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2022 Ennio Sesana
  Copyright (c) 2023-2024 Terje Io (added SD card, ModBus and MPG options)

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

#include "use_i2s_out.h"

#define BOARD_NAME "MKS Tinybee V1.0"
#define BOARD_URL "https://github.com/makerbase-mks/MKS-TinyBee"

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if KEYPAD_ENABLE == 1
#error "No free pins for I2C keypad!"
#endif

#define SERIAL1_PORT // RX: 16, TX: 17

#define I2S_OUT_BCK         GPIO_NUM_25
#define I2S_OUT_WS          GPIO_NUM_26
#define I2S_OUT_DATA        GPIO_NUM_27

#define X_STEP_PIN          I2SO(1) 
#define X_DIRECTION_PIN     I2SO(2) 
#define X_ENABLE_PIN        I2SO(0) 
#define X_LIMIT_PIN         GPIO_NUM_33

#define Y_STEP_PIN          I2SO(4)
#define Y_DIRECTION_PIN     I2SO(5)
#define Y_ENABLE_PIN        I2SO(3)
#define Y_LIMIT_PIN         GPIO_NUM_32

#define Z_STEP_PIN          I2SO(7)
#define Z_DIRECTION_PIN     I2SO(8)
#define Z_ENABLE_PIN        I2SO(6)
#define Z_LIMIT_PIN         GPIO_NUM_22

#if PROBE_ENABLE || N_ABC_MOTORS == 0
#define AUXINPUT1_PIN       GPIO_NUM_35 // MT_DET
#endif

#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE        // E0
#define M3_STEP_PIN         I2SO(10)
#define M3_DIRECTION_PIN    I2SO(11)
#define M3_ENABLE_PIN       I2SO(9)
#ifndef AUXINPUT1_PIN
#define M3_LIMIT_PIN        GPIO_NUM_35 // MT_DET
#elif SDCARD_ENABLE
#define M3_LIMIT_PIN        GPIO_NUM_12 // EXP2
#else
#define M3_LIMIT_PIN        GPIO_NUM_19 // EXP2
#endif
#endif

#if N_ABC_MOTORS == 2
#define M4_AVAILABLE        // E1
#define M4_STEP_PIN         I2SO(13)
#define M4_DIRECTION_PIN    I2SO(14)
#define M4_ENABLE_PIN       I2SO(12)
#if SDCARD_ENABLE
#define M4_LIMIT_PIN        GPIO_NUM_14 // EXP2
#else
#define M4_LIMIT_PIN        GPIO_NUM_18 // EXP2
#endif
#endif

#define AUXOUTPUT0_PIN      GPIO_NUM_2 // Spindle PWM
#define AUXOUTPUT1_PIN      I2SO(18)   // Spindle direction, HE1
#define AUXOUTPUT2_PIN      I2SO(17)   // Spindle enable, HE0
#define AUXOUTPUT3_PIN      I2SO(19)   // Coolant flood, FAN1
#define AUXOUTPUT4_PIN      I2SO(20)   // Coolant mist, FAN2

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PIN           GPIO_NUM_39 // TB
#define AUXINPUT2_PIN           GPIO_NUM_36 // Feed hold
#define AUXINPUT3_PIN           GPIO_NUM_34 // Cycle start
#if MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED
#define MODBUS_DIRECTION_PIN    GPIO_NUM_13 // EXP_1
#else
#define AUXINPUT4_PIN           GPIO_NUM_13 // EXP_1
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.

// Reset/EStop is connected to MCU reset pin.
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN           AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT3_PIN
#endif

// Define probe switch input pin.
#if PROBE_ENABLE && defined(AUXINPUT1_PIN)
#define PROBE_PIN               AUXINPUT1_PIN // MT_DET
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN // TB
#endif

#if MPG_ENABLE == 1 && defined(AUXINPUT4_PIN)
#define MPG_MODE_PIN            AUXINPUT4_PIN // EXP_1
#endif

#if SDCARD_ENABLE
#define SPI_MISO_PIN            GPIO_NUM_19
#define SPI_MOSI_PIN            GPIO_NUM_23
#define SPI_SCK_PIN             GPIO_NUM_18
#define SD_CS_PIN               GPIO_NUM_5
#endif
