/*
  ooznest_cnc_map.h - grblHAL pin map for Ooznest CNC Controller

  GPIO > Function
  0  > Macro Button
  1  > Modbus Direction (RS485 TX Enable)
  2  > Stepper Enable (All axes)
  3  > SDA (I2C)
  4  > SCL (I2C)
  5  > SD Card CS
  6  > Ethernet CS (W5500)
  7  > Ethernet Interrupt (W5500)
  8  > I2S BCK
  9  > I2S WS (LCK)
  10 > I2S DATA
  11 > Primary Spindle Enable
  12 > Primary Spindle PWM
  13 > Secondary Spindle PWM (Laser)
  14 > E-Stop (Input)
  15 > AUX Output 1
  16 > Toolsetter (Probe 2)
  17 > AUX Output 2
  18 > AUX Output 3
  21 > Probe 1
  35 > SPI MOSI
  36 > SPI SCK
  37 > SPI MISO
  38 > X Limit
  39 > Y1 Limit
  40 > Y2 Limit
  41 > Z Limit
  42 > X Motor Fault
  43 > UART1 TX (Modbus)
  44 > UART1 RX (Modbus)
  45 > Y1 Motor Fault
  46 > Y2 Motor Fault
  47 > Z Motor Fault
  48 > RGB LED (Neopixel)
*/

#ifndef CONFIG_IDF_TARGET_ESP32S3
#error "This board has ESP32-S3 processor, select a corresponding build!"
#endif

#define BOARD_NAME "Ooznest-CNC"
#define BOARD_URL "https://ooznest.co.uk"
#define HAS_BOARD_INIT

#include "use_i2s_out.h"

// I2S Configuration (Shift Register used for Stepper Signals)
#define I2S_OUT_BCK             GPIO_NUM_8
#define I2S_OUT_WS              GPIO_NUM_9
#define I2S_OUT_DATA            GPIO_NUM_10

/*
I2S Step/Dir Signals (Q0-Q7) mapping:
Q0=X_STP
Q1=X_DIR
Q2=Y1_STP
Q3=Y1_DIR
Q4=Y2_STP
Q5=Y2_DIR
Q6=Z_STP
Q7=Z_DIR
*/
#define X_STEP_PIN              I2SO(0)
#define X_DIRECTION_PIN         I2SO(1)

#define Y_STEP_PIN              I2SO(2)
#define Y_DIRECTION_PIN         I2SO(3)

// Renamed from M3 to Y2 for Y_AUTO_SQUARE support?
#define Y2_STEP_PIN             I2SO(4)
#define Y2_DIRECTION_PIN        I2SO(5)

#define Z_STEP_PIN              I2SO(6)
#define Z_DIRECTION_PIN         I2SO(7)

#define STEPPERS_ENABLE_PIN     GPIO_NUM_2

// Limit Switch Pins
#define X_LIMIT_PIN             GPIO_NUM_38
#define Y_LIMIT_PIN             GPIO_NUM_39
#define Y2_LIMIT_PIN            GPIO_NUM_40
#define Z_LIMIT_PIN             GPIO_NUM_41

#define RESET_PIN               GPIO_NUM_14

// I2C - onboard MCP4728 and INA219
#undef I2C_ENABLE
#define I2C_ENABLE              1
#define I2C_SDA                 GPIO_NUM_3
#define I2C_SCL                 GPIO_NUM_4
#define I2C_PORT                I2C_NUM_0
#define I2C_CLOCK               100000

// SPI
#define SPI_MOSI_PIN            GPIO_NUM_35
#define SPI_SCK_PIN             GPIO_NUM_36
#define SPI_MISO_PIN            GPIO_NUM_37
#define SD_CS_PIN               GPIO_NUM_5

// enet.c overrides
#define INPUT_GPIO_CS 5         GPIO_NUM_6
#define INPUT_GPIO_INTERRUPT    GPIO_NUM_7
#define INPUT_GPIO_MOSI         GPIO_NUM_35
#define INPUT_GPIO_SCLK         GPIO_NUM_36
#define INPUT_GPIO_MISO         GPIO_NUM_37


// Spindles
// Primary Spindle = PWM + IOT Relay + 0-10v. Enable and PWM, no DIR
#define SPINDLE_ENABLE_PIN      GPIO_NUM_11
#define SPINDLE_PWM_PIN         GPIO_NUM_12

// Spindles
//Secondary Spindle = Laser Port. No Dir, No Enable
#define SPINDLE1_ENABLE_PIN     GPIO_NUM_NC
#define SPINDLE1_PWM_PIN        GPIO_NUM_13

// AUX
#define AUXOUTPUT0_PIN          GPIO_NUM_1  // Modbus Dir
#define AUXOUTPUT1_PIN          GPIO_NUM_15 // Coolant / Aux Mosfet
#define AUXOUTPUT2_PIN          GPIO_NUM_17 // Aux signal on GPIO header
#define AUXOUTPUT3_PIN          GPIO_NUM_18 // Aux signal on GPIO header
#define AUXINPUT0_PIN           GPIO_NUM_0  // Mode Button
#define AUXINPUT1_PIN           GPIO_NUM_21 // Probe
#define AUXINPUT2_PIN           GPIO_NUM_16 // TLS / Probe 2

// Probes
#define PROBE_PIN               AUXINPUT1_PIN
#define TOOLSETTER_PIN          AUXINPUT2_PIN

// UART1 (Modbus)
#define SERIAL1_PORT            1
#define UART1_RX_PIN            GPIO_NUM_44
#define UART1_TX_PIN            GPIO_NUM_43

// Neopixel
#define LED_PIN                 GPIO_NUM_48
#define NEOPIXEL_DMA_ENABLE     1
#define NEOPIXELS_NUM           3 // 3 LEDs onboard, port for more offboard


// Not created yet
// - Motor Fault
// - E-Stop Input
