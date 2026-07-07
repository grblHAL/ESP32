/*
  pibot_5_88_ultra_map.h - An esp32-s3 embedded CNC Controller 

  Part of grblHAL

  Copyright (c) 2026 @luc-github

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

#ifndef CONFIG_IDF_TARGET_ESP32S3
#error "This board has ESP32-S3 processor, select a corresponding build!"
#endif

#include "use_i2s_out.h"

#define BOARD_NAME "PiBot V5.88 Ultra"
#define BOARD_URL " https://www.pibot.com/cnc-laser-electronics/pibot-fluidnc-grblhal-esp32-s3-6-1-axis-cnc-controller-v5-88-ultra"

// SPI
#define SPI_MISO_PIN            GPIO_NUM_2
#define SPI_MOSI_PIN            GPIO_NUM_1
#define SPI_SCK_PIN             GPIO_NUM_21

// I2S 
#define I2S_OUT_BCK             GPIO_NUM_16
#define I2S_OUT_DATA            GPIO_NUM_17
#define I2S_OUT_WS              GPIO_NUM_18

// SD Card
#if SDCARD_ENABLE
#define SD_CS_PIN               GPIO_NUM_9
#endif

// I2C (Oled)
#if I2C_ENABLE
#define I2C_PORT                I2C_NUM_0
#define I2C_SDA                 GPIO_NUM_14
#define I2C_SCL                 GPIO_NUM_13
#define I2C_CLOCK               100000
#endif

// Axis
#define X_STEP_PIN              I2SO(2)
#define X_DIRECTION_PIN         I2SO(1)
#define X_ENABLE_PIN            I2SO(0)
#define X_LIMIT_PIN             GPIO_NUM_42
#define X_LIMIT_PIN_MAX         GPIO_NUM_41
#define MOTOR_CSX_PIN           I2SO(3)

#define Y_STEP_PIN              I2SO(5)
#define Y_DIRECTION_PIN         I2SO(4)
#define Y_ENABLE_PIN            I2SO(7)
#define Y_LIMIT_PIN             GPIO_NUM_40
#define Y_LIMIT_PIN_MAX         GPIO_NUM_39
#define MOTOR_CSY_PIN           I2SO(6)


#define Z_STEP_PIN              I2SO(10)
#define Z_DIRECTION_PIN         I2SO(9)
#define Z_ENABLE_PIN            I2SO(8)
#define Z_LIMIT_PIN             GPIO_NUM_38
#define MOTOR_CSZ_PIN           I2SO(11)

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN             I2SO(13)
#define M3_DIRECTION_PIN        I2SO(12)
#define M3_ENABLE_PIN           I2SO(15)
#define MOTOR_CSM3_PIN          I2SO(14)
#if M3_LIMIT_ENABLE
#define M3_LIMIT_PIN            GPIO_NUM_37
#endif
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 2
#define M4_AVAILABLE
#define M4_STEP_PIN             I2SO(18)
#define M4_DIRECTION_PIN        I2SO(17)
#define M4_ENABLE_PIN           I2SO(16)
#define MOTOR_CSM4_PIN          I2SO(19)
#if M4_LIMIT_ENABLE
#define M4_LIMIT_PIN            GPIO_NUM_36
#endif
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PIN             I2SO(21)
#define M5_DIRECTION_PIN        I2SO(20)
#define M5_ENABLE_PIN           I2SO(23)
#define MOTOR_CSM5_PIN          I2SO(22)
#if M5_LIMIT_ENABLE
#define M5_LIMIT_PIN            GPIO_NUM_48
#endif
#endif


// AUX
#define AUXOUTPUT0_PIN          GPIO_NUM_4 // Aux signal on GPIO header
#define AUXOUTPUT1_PIN          GPIO_NUM_5 // Aux signal on GPIO header
#define AUXOUTPUT2_PIN          GPIO_NUM_10 // Coolant flood
#define AUXOUTPUT3_PIN          GPIO_NUM_15 // Coolant mist
#define AUXOUTPUT4_PIN          GPIO_NUM_45 // Spindle PWM
#define AUXOUTPUT5_PIN          GPIO_NUM_46 // Laser|
#define AUXOUTPUT6_PIN          GPIO_NUM_6  // output
#define AUXOUTPUT7_PIN          GPIO_NUM_7  // +10v forward
#define AUXOUTPUT8_PIN          GPIO_NUM_8  // +10v reverse

#define AUXINPUT0_PIN           GPIO_NUM_0  // Mode Button
#define AUXINPUT1_PIN           GPIO_NUM_47 // Probe
#define AUXINPUT2_PIN           GPIO_NUM_3  // Tool Setter (requires jumper selection first)
#if !M5_LIMIT_ENABLE
#define AUXINPUT4_PIN           GPIO_NUM_48 // Door pin
#endif

#if !M4_LIMIT_ENABLE
#define AUXINPUT5_PIN           GPIO_NUM_36 // TBD
#endif


// Spindles
// reference: (ttps://wiki.pibot.com/doku.php?id=pibot_cnc_laser_series:v588_ultra:pinout_reference:start#pb1

// Primary Spindle = PWM + IOT Relay + 0-10v. Enable and PWM, no DIR
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN AUXOUTPUT4_PIN
#endif

#if (DRIVER_SPINDLE_ENABLE & SPINDLE_ENA)
#define SPINDLE_ENABLE_PIN      SPINDLE_ENABLE_DUMMY_PIN
#endif


//Secondary Spindle = Laser Port
//10V:
// forward_pin: AUXOUTPUT7_PIN
// reverse_pin: AUXOUTPUT8_PIN
// output_pin:  AUXOUTPUT6_PIN

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
#define DRIVER_SPINDLE1_ENABLE SPINDLE_PWM
#define SPINDLE1_PWM_PIN        AUXOUTPUT5_PIN
#endif

// Inputs

// Probe
#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT1_PIN
#endif

// Tool Setter
#if TOOLSETTER_ENABLE
#define TOOLSETTER_PIN          AUXINPUT2_PIN
#endif

// Door
#if SAFETY_DOOR_ENABLE
#if !M5_LIMIT_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT4_PIN
#endif
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
   #define COOLANT_FLOOD_PIN     AUXOUTPUT2_PIN 
 #endif
#if COOLANT_ENABLE & COOLANT_MIST
  #define COOLANT_MIST_PIN       AUXOUTPUT3_PIN 
#endif


// UART1 (Modbus)
#ifdef ADD_SERIAL1
#define SERIAL1_PORT            1
#define UART1_RX_PIN            GPIO_NUM_11
#define UART1_TX_PIN            GPIO_NUM_12
#endif

// UART2 (pendant)
#ifdef ADD_SERIAL2
#define SERIAL2_PORT
#define UART1_RX_PIN            GPIO_NUM_35
#define UART1_TX_PIN            GPIO_NUM_0
#endif


