/*
  ooznest_cnc_map.h - grblHAL pin map for Ooznest CNC Controller

  GPIO > Function
  0  > Macro Button

  48 > RGB LED (Neopixel)
*/

#ifndef CONFIG_IDF_TARGET_ESP32S3
#error "This board has ESP32-S3 processor, select a corresponding build!"
#endif

#define BOARD_NAME "Ooznest-Motion-Control-Core"
#define BOARD_URL "https://ooznest.co.uk"
#define HAS_BOARD_INIT

#define USE_EXPANDERS
#define SPI_CS0_PIN             GPIO_NUM_8
#define HC595_CS_PIN            SPI_CS0_PIN

#define X_STEP_PIN              GPIO_NUM_38
#define X_DIRECTION_PIN         GPIO_NUM_42

#define Y_STEP_PIN              GPIO_NUM_39
#define Y_DIRECTION_PIN         GPIO_NUM_45

#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PIN             GPIO_NUM_40
#define M3_DIRECTION_PIN        GPIO_NUM_46
#define M3_LIMIT_PIN            GPIO_NUM_11
#endif

#define Z_STEP_PIN              GPIO_NUM_41
#define Z_DIRECTION_PIN         GPIO_NUM_47

#define STEPPERS_ENABLE_PORT    EXPANDER_PORT
#define STEPPERS_ENABLE_PIN     3

// Limit Switch Pins
#define X_LIMIT_PIN             GPIO_NUM_9
#define Y_LIMIT_PIN             GPIO_NUM_10
#define Z_LIMIT_PIN             GPIO_NUM_12

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
#define INPUT_GPIO_CS           GPIO_NUM_6
#define INPUT_GPIO_INTERRUPT    GPIO_NUM_7
#define INPUT_GPIO_MOSI         GPIO_NUM_35
#define INPUT_GPIO_SCLK         GPIO_NUM_36
#define INPUT_GPIO_MISO         GPIO_NUM_37

// AUX
#define AUXOUTPUT0_PIN          GPIO_NUM_1 // Aux signal on GPIO header
#define AUXOUTPUT1_PIN          GPIO_NUM_2 // Aux signal on GPIO header
#define AUXOUTPUT2_PIN          GPIO_NUM_14 // Spindle 1 PWM
#define AUXOUTPUT3_PIN          GPIO_NUM_13 // Spindle PWM

#define AUXINPUT0_PIN           GPIO_NUM_0  // Mode Button
#define AUXINPUT1_PIN           GPIO_NUM_17 // Probe
#define AUXINPUT2_PIN           GPIO_NUM_16 // TLS / Probe 2
#define AUXINPUT3_PIN           GPIO_NUM_15 // Reset / E-Stop
#define AUXINPUT4_PIN           GPIO_NUM_18 // Door Input
#define AUXINPUT5_PIN           GPIO_NUM_21 // Motor Faults pin

// Spindles

// Primary Spindle = PWM + IOT Relay + 0-10v. Enable and PWM, no DIR
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     EXPANDER_PORT
#define SPINDLE_ENABLE_PIN      0
#endif

//Secondary Spindle = Laser Port. No Dir, No Enable
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
#undef DRIVER_SPINDLE1_ENABLE
#define DRIVER_SPINDLE1_ENABLE SPINDLE_PWM
#define SPINDLE1_PWM_PIN        AUXOUTPUT2_PIN
#endif

#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT3_PIN
#endif

// Probes
#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT1_PIN
#endif
#if TOOLSETTER_ENABLE
#define TOOLSETTER_PIN          AUXINPUT2_PIN
#endif

// UART1 (Modbus)
#define SERIAL1_PORT            1
#define UART1_RX_PIN            GPIO_NUM_44
#define UART1_TX_PIN            GPIO_NUM_43

#if MODBUS_ENABLE
#define MODBUS_RTU_STREAM       1
#undef MODBUS_ENABLE
#define MODBUS_ENABLE           (MODBUS_RTU_ENABLED|MODBUS_RTU_DIR_ENABLED)
#define MODBUS_DIR_AUX          6
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT4_PIN
#endif

// // Define flood and mist coolant enable output pins.
// #if COOLANT_ENABLE & COOLANT_MIST
//   #define COOLANT_MIST_PORT      EXPANDER_PORT
//   #define COOLANT_MIST_PIN       1
// #endif
// #if COOLANT_ENABLE & COOLANT_FLOOD
//   #define COOLANT_FLOOD_PORT      EXPANDER_PORT
//   #define COOLANT_FLOOD_PIN       4
// #endif


// Neopixel
#define LED_PIN                 GPIO_NUM_48
#define NEOPIXEL_SPI

// Notor Fault
#define MOTOR_FAULT_PIN         21



// Not created yet
// - Motor Fault
// - Door Input
