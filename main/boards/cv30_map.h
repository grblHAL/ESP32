/*
  cv30_map.h - driver code for ESP32-S2

  Board map for the Creality "CV-30" laser control box that ships with the
  Creality 3D-printer laser module kits (1.6/5/10 W). ESP32-S2, 4 MB flash,
  native USB, microSD, two stepper drivers, laser PWM + enable.

  Pins recovered from Creality's own firmware (CV-30-3DPrint-V3_0_24,
  Grbl_Esp32 1.3a): see cv30/pinmap.md.

  Part of grblHAL

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

#if !CONFIG_IDF_TARGET_ESP32S2
#error "The CV-30 laser box has an ESP32-S2."
#endif

#if N_ABC_MOTORS > 0
#error "The CV-30 laser box has two stepper drivers (X and Y) only."
#endif

#define BOARD_NAME "Creality CV-30 laser box"

// Steppers. Z has no driver on this board; it is left without pins.
#define X_STEP_PIN              GPIO_NUM_36
#define X_DIRECTION_PIN         GPIO_NUM_35
#define Y_STEP_PIN              GPIO_NUM_7
#define Y_DIRECTION_PIN         GPIO_NUM_6
// High = drivers off, as in the stock firmware; grblHAL's default
// $4 (enable invert on all axes) drives it low when enabled.
#define STEPPERS_ENABLE_PIN     GPIO_NUM_16

// Min end stops only (inputs with pull-ups).
#define X_LIMIT_PIN             GPIO_NUM_40
#define Y_LIMIT_PIN             GPIO_NUM_42

// Laser. The stock firmware writes duty = 1023 - power, i.e. the PWM output
// is inverted on this board, and drives GPIO38 high to switch the laser off.
// Both inversions are set as defaults in CMakeLists.txt (BOARD_CV30).
#define AUXOUTPUT0_PIN          GPIO_NUM_37 // Laser PWM
#define AUXOUTPUT1_PIN          GPIO_NUM_38 // Laser enable (active low)
#define AUXOUTPUT2_PIN          GPIO_NUM_9  // Buzzer
#define AUXOUTPUT3_PIN          GPIO_NUM_34 // Status LED

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT1_PIN
#endif

// Front button (input with pull-up). Free for now: available as an aux input.
#define AUXINPUT0_PIN           GPIO_NUM_33

#if SDCARD_ENABLE
#define SPI_MISO_PIN            GPIO_NUM_2
#define SPI_MOSI_PIN            GPIO_NUM_15
#define SPI_SCK_PIN             GPIO_NUM_14
#define SD_CS_PIN               GPIO_NUM_13
#endif
