/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2018-2024 Terje Io

  Some parts
   Copyright (c) 2011-2015 Sungeun K. Jeon
   Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>

#include "./driver.h"
#include "uart_serial.h"
#include "timers.h"
#include "nvs.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_ota_ops.h"
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "hal/rmt_ll.h"
#include "driver/i2c.h"
#include "hal/gpio_types.h"
#include "xtensa/core-macros.h"

#include "grbl/task.h"
#include "grbl/state_machine.h"
#include "grbl/motor_pins.h"
#include "grbl/machine_limits.h"
#include "grbl/pin_bits_masks.h"
#if NVSDATA_BUFFER_ENABLE
#include "grbl/nvs_buffer.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/clk.h"
#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif // USB_SERIAL_CDC
#endif

#if WIFI_ENABLE
#include "wifi.h"
#endif

#if ETHERNET_ENABLE
#include "enet.h"
#endif

#if BLUETOOTH_ENABLE
#include "bluetooth.h"
#endif

#if SDCARD_ENABLE
#include "spi.h"
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

#if LITTLEFS_ENABLE
#include "littlefs_hal.h"
#include "sdcard/fs_littlefs.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#ifndef MPG_SHARE_TX
#define MPG_SHARE_TX 0
#endif

#if PWM_RAMPED

#define SPINDLE_RAMP_STEP_INCR 20 // timer compare register change per ramp step
#define SPINDLE_RAMP_STEP_TIME 2  // ms

typedef struct {
    volatile uint32_t ms_cfg;
    volatile uint32_t ms_count;
    uint32_t pwm_current;
    uint32_t pwm_target;
    uint32_t pwm_step;
} pwm_ramp_t;

#endif

typedef struct {
    uint32_t pwm_max_value;
    bool pwm_enabled;
    spindle_pwm_t spindle_pwm;
    ledc_timer_config_t timer;
    ledc_channel_config_t channel;
#if PWM_RAMPED
    pwm_ramp_t ramp;
#endif
    spindle1_pwm_settings_t *config;
} pwm_spindle_t;

#if DRIVER_SPINDLE_ENABLE

static spindle_id_t spindle_id = -1;

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

static pwm_spindle_t pwm_spindle = {
    .pwm_enabled = false,
    .timer = {
#if CONFIG_IDF_TARGET_ESP32S3
        .speed_mode = LEDC_LOW_SPEED_MODE,
#else
        .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000
    },
    .channel = {
        .gpio_num = SPINDLE_PWM_PIN,
#if CONFIG_IDF_TARGET_ESP32S3
        .speed_mode = LEDC_SPEED_MODE_MAX,
#else
        .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
        .hpoint = 0
    }
};

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE

static spindle_id_t spindle1_id = -1;

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

static pwm_spindle_t pwm_spindle1 = {
    .pwm_enabled = false,
    .timer = {
#if CONFIG_IDF_TARGET_ESP32S3
        .speed_mode = LEDC_LOW_SPEED_MODE,
#else
        .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 5000
    },
    .channel = {
        .gpio_num = SPINDLE1_PWM_PIN,
#if CONFIG_IDF_TARGET_ESP32S3
        .speed_mode = LEDC_SPEED_MODE_MAX,
#else
        .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
        .hpoint = 0
    }
};

#endif // DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
#endif // DRIVER_SPINDLE1_ENABLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// prescale step counter to 20Mhz
#define STEPPER_DRIVER_PRESCALER 4

static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
#ifdef X_LIMIT_PIN
    { .id = Input_LimitX,       .pin = X_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,     .pin = X2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,   .pin = X_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
#endif
#ifdef Y_LIMIT_PIN
    { .id = Input_LimitY,       .pin = Y_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,     .pin = Y2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,   .pin = Y_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
#endif
#ifdef Z_LIMIT_PIN
    { .id = Input_LimitZ,       .pin = Z_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,     .pin = Z2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,   .pin = Z_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
#endif
#ifdef A_LIMIT_PIN
    { .id = Input_LimitA,       .pin = A_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef A_LIMIT_PIN_MAX
    { .id = Input_LimitA_Max,   .pin = A_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,       .pin = B_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN_MAX
    { .id = Input_LimitB_Max,   .pin = B_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,       .pin = C_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN_MAX
    { .id = Input_LimitC_Max,   .pin = C_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
#endif
#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
    { .id = Input_SdCardDetect, .pin = SD_DETECT_PIN,     .group = PinGroup_SdCard },
#endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,         .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,         .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,         .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT3_PIN
    { .id = Input_Aux3,         .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT4_PIN
    { .id = Input_Aux4,         .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT5_PIN
    { .id = Input_Aux5,         .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT6_PIN
    { .id = Input_Aux6,         .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT7_PIN
    { .id = Input_Aux7,         .pin = AUXINPUT7_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT0_ANALOG_PIN
    { .id = Input_Analog_Aux0,  .pin = AUXINPUT0_ANALOG_PIN, .group = PinGroup_AuxInputAnalog },
#endif
#ifdef AUXINPUT1_ANALOG_PIN
    { .id = Input_Analog_Aux1,  .pin = AUXINPUT1_ANALOG_PIN, .group = PinGroup_AuxInputAnalog }
#endif
};

const size_t inputPinsCount = sizeof(inputpin) / sizeof(input_signal_t);

static output_signal_t outputpin[] = {
    { .id = Output_StepX,          .pin = X_STEP_PIN,            .group = PinGroup_StepperStep },
    { .id = Output_StepY,          .pin = Y_STEP_PIN,            .group = PinGroup_StepperStep },
#ifdef Z_STEP_PIN
    { .id = Output_StepZ,          .pin = Z_STEP_PIN,            .group = PinGroup_StepperStep },
#endif
#ifdef A_STEP_PIN
    { .id = Output_StepA,          .pin = A_STEP_PIN,            .group = PinGroup_StepperStep },
#endif
#ifdef B_STEP_PIN
    { .id = Output_StepB,          .pin = B_STEP_PIN,            .group = PinGroup_StepperStep },
#endif
#ifdef C_STEP_PIN
    { .id = Output_StepC,          .pin = C_STEP_PIN,            .group = PinGroup_StepperStep },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,        .pin = X2_STEP_PIN,           .group = PinGroup_StepperStep },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,        .pin = Y2_STEP_PIN,           .group = PinGroup_StepperStep },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,        .pin = Z2_STEP_PIN,           .group = PinGroup_StepperStep },
#endif

#if defined(STEPPERS_ENABLE_PIN) && !defined(STEPPERS_ENABLE_PORT)
    { .id = Output_StepperEnable,  .pin = STEPPERS_ENABLE_PIN,   .group = PinGroup_StepperEnable },
#endif
#if defined(XY_ENABLE_PIN) && !defined(XY_ENABLE_PORT)
    { .id = Output_StepperEnableXY, .pin = XY_ENABLE_PIN,         .group = PinGroup_StepperEnable },
#else
  #if defined(X_ENABLE_PIN) && !defined(X_ENABLE_PORT)
    { .id = Output_StepperEnableX, .pin = X_ENABLE_PIN,          .group = PinGroup_StepperEnable },
  #endif
  #if defined(Y_ENABLE_PIN) && !defined(Y_ENABLE_PORT)
    { .id = Output_StepperEnableY, .pin = Y_ENABLE_PIN,          .group = PinGroup_StepperEnable },
  #endif
#endif
#if defined(Z_ENABLE_PIN) && !defined(Z_ENABLE_PORT)
    { .id = Output_StepperEnableZ, .pin = Z_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(A_ENABLE_PIN) && !defined(A_ENABLE_PORT)
    { .id = Output_StepperEnableA, .pin = A_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(B_ENABLE_PIN) && !defined(B_ENABLE_PORT)
    { .id = Output_StepperEnableB, .pin = B_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(C_ENABLE_PIN) && !defined(C_ENABLE_PORT)
    { .id = Output_StepperEnableC, .pin = C_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(X2_ENABLE_PIN) && !defined(X2_ENABLE_PORT)
    { .id = Output_StepperEnableX, .pin = X2_ENABLE_PIN,         .group = PinGroup_StepperEnable },
#endif
#if defined(Y2_ENABLE_PIN) && !defined(Y2_ENABLE_PORT)
    { .id = Output_StepperEnableY, .pin = Y2_ENABLE_PIN,         .group = PinGroup_StepperEnable },
#endif
#if defined(Z2_ENABLE_PIN) && !defined(Z2_ENABLE_PORT)
    { .id = Output_StepperEnableZ, .pin = Z2_ENABLE_PIN,         .group = PinGroup_StepperEnable },
#endif
    { .id = Output_DirX,           .pin = X_DIRECTION_PIN,       .group = PinGroup_StepperDir },
    { .id = Output_DirY,           .pin = Y_DIRECTION_PIN,       .group = PinGroup_StepperDir },
#ifdef Z_DIRECTION_PIN
    { .id = Output_DirZ,           .pin = Z_DIRECTION_PIN,       .group = PinGroup_StepperDir },
#endif
#ifdef A_AXIS
    { .id = Output_DirA,           .pin = A_DIRECTION_PIN,       .group = PinGroup_StepperDir },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,           .pin = B_DIRECTION_PIN,       .group = PinGroup_StepperDir },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,           .pin = C_DIRECTION_PIN,       .group = PinGroup_StepperDir },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,         .pin = X2_DIRECTION_PIN,      .group = PinGroup_StepperDir },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,         .pin = Y2_DIRECTION_PIN,      .group = PinGroup_StepperDir },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,         .pin = Z2_DIRECTION_PIN,      .group = PinGroup_StepperDir },
#endif
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,   .pin = MOTOR_CS_PIN,      .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSX_PIN
    { .id = Output_MotorChipSelectX,  .pin = MOTOR_CSX_PIN,     .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSY_PIN
    { .id = Output_MotorChipSelectY,  .pin = MOTOR_CSY_PIN,     .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSZ_PIN
    { .id = Output_MotorChipSelectZ,  .pin = MOTOR_CSZ_PIN,     .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM3_PIN
    { .id = Output_MotorChipSelectM3, .pin = MOTOR_CSM3_PIN,    .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM4_PIN
    { .id = Output_MotorChipSelectM4, .pin = MOTOR_CSM4_PIN,    .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM5_PIN
    { .id = Output_MotorChipSelectM5, .pin = MOTOR_CSM5_PIN,    .group = PinGroup_MotorChipSelect },
#endif
#ifdef SD_CS_PIN
    { .id = Output_SdCardCS,          .pin = SD_CS_PIN,        .group = PinGroup_SdCard },
#endif
#ifdef MODBUS_DIRECTION_PIN
    { .id = Output_Aux0,           .pin = MODBUS_DIRECTION_PIN,  .group = PinGroup_AuxOutput },
#elif defined(AUXOUTPUT0_PIN)
    { .id = Output_Aux0,           .pin = AUXOUTPUT0_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PIN
    { .id = Output_Aux1,           .pin = AUXOUTPUT1_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PIN
    { .id = Output_Aux2,           .pin = AUXOUTPUT2_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PIN
    { .id = Output_Aux3,           .pin = AUXOUTPUT3_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PIN
    { .id = Output_Aux4,           .pin = AUXOUTPUT4_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PIN
    { .id = Output_Aux5,           .pin = AUXOUTPUT5_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PIN
    { .id = Output_Aux6,           .pin = AUXOUTPUT6_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PIN
    { .id = Output_Aux7,           .pin = AUXOUTPUT7_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT8_PIN
    { .id = Output_Aux8,           .pin = AUXOUTPUT8_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT9_PIN
    { .id = Output_Aux9,           .pin = AUXOUTPUT9_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT0_PWM_PIN
    { .id = Output_Analog_Aux0,    .pin = AUXOUTPUT0_PWM_PIN,    .group = PinGroup_AuxOutputAnalog, .mode = {PINMODE_PWM} },
#endif
#ifdef AUXOUTPUT0_ANALOG_PIN
    { .id = Output_Analog_Aux0,    .pin = AUXOUTPUT0_ANALOG_PIN, .group = PinGroup_AuxOutputAnalog },
#endif
#ifdef AUXOUTPUT1_PWM_PIN
    { .id = Output_Analog_Aux1,    .pin = AUXOUTPUT1_PWM_PIN,    .group = PinGroup_AuxOutputAnalog, .mode = {PINMODE_PWM} },
#endif
#ifdef AUXOUTPUT1_ANALOG_PIN
    { .id = Output_Analog_Aux1,    .pin = AUXOUTPUT1_ANALOG_PIN, .group = PinGroup_AuxOutputAnalog }
#endif
};

static const size_t outputPinsCount = sizeof(outputpin) / sizeof(output_signal_t);

static bool IOInitDone = false;
static uint32_t t_min_period;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static pin_group_pins_t limit_inputs = {0};
static on_execute_realtime_ptr on_execute_realtime;
#if STEP_INJECT_ENABLE
static struct {
    uint32_t length;
    uint32_t delay;
    axes_signals_t out;
    struct {
        hal_timer_t timer;
        axes_signals_t claimed;
        volatile axes_signals_t axes;
        volatile axes_signals_t out;
    } inject;
} step_pulse = {0};
#endif

#ifdef USE_EXPANDERS
static xbar_t *iox_out[8] = {};
#endif

#ifdef LED_PIN
neopixel_cfg_t neopixel = { .intensity = 255 };
void neopixels_write (void);
#endif

#ifdef SAFETY_DOOR_PIN
static pin_debounce_t debounce = {0};
static input_signal_t *safety_door = NULL;
#endif
static void aux_irq_handler (uint8_t port, bool state);

#if I2C_STROBE_ENABLE

static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok;

    if((ok = irq == IRQ_I2C_Strobe && i2c_strobe.callback == NULL))
        i2c_strobe.callback = handler;

    return ok;
}

#endif

// Interrupt handler prototypes
#if ETHERNET_ENABLE
static void gpio_limit_isr (void *signal);
static void gpio_control_isr (void *signal);
static void gpio_aux_isr (void *signal);
#if MPG_ENABLE == 1
static void gpio_mpg_isr (void *signal);
#endif
#if I2C_STROBE_ENABLE
static void gpio_i2c_strobe_isr (void *signal);
#endif
#else
static void gpio_isr (void *arg);
#endif
static void stepper_driver_isr (void *arg);

static TimerHandle_t xDelayTimer = NULL;

void vTimerCallback (TimerHandle_t xTimer)
{
    void (*callback)(void) = (void (*)(void))pvTimerGetTimerID(xTimer);

    if(callback)
        callback();

    xTimerDelete(xDelayTimer, 3);
    xDelayTimer = NULL;
}

IRAM_ATTR static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(callback) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        if(ms) {
            xDelayTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(ms), pdFALSE, callback, vTimerCallback);
            xTimerStartFromISR(xDelayTimer, &xHigherPriorityTaskWoken);
            if(xHigherPriorityTaskWoken)
                portYIELD_FROM_ISR();
        } else
            callback();
    } else {
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        while(ms) {
            vTaskDelay(pdMS_TO_TICKS(2));
            grbl.on_execute_delay(state_get());
            ms -= ms > 1 ? 2 : 1;
        }
    }
}

// Enable/disable steppers
static void stepperEnable (axes_signals_t enable, bool hold)
{
    enable.mask ^= settings.steppers.enable_invert.mask;

#if !TRINAMIC_MOTOR_ENABLE
 #if defined(STEPPERS_ENABLE_PIN)
  #if STEPPERS_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(STEPPERS_ENABLE_PIN, enable.x);
  #else
    DIGITAL_OUT(STEPPERS_ENABLE_PIN, enable.x);
  #endif
 #else
  #ifdef XY_ENABLE_PIN
   #if XY_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(XY_ENABLE_PIN, enable.x || enable.y);
   #else
    DIGITAL_OUT(XY_ENABLE_PIN, enable.x || enable.y);
   #endif
  #endif
  #ifdef X_ENABLE_PIN
   #if X_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(X_ENABLE_PIN, enable.x);
   #else
    DIGITAL_OUT(X_ENABLE_PIN, enable.x);
   #endif
  #endif
  #ifdef X2_ENABLE_PIN
    DIGITAL_OUT(X2_ENABLE_PIN, enable.x);
  #endif
  #ifdef Y_ENABLE_PIN
   #if Y_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(Y_ENABLE_PIN, enable.y);
   #else
    DIGITAL_OUT(Y_ENABLE_PIN, enable.y);
   #endif
  #endif
  #ifdef Y2_ENABLE_PIN
   #if Y2_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(XY_ENABLE_PIN, enable.y);
   #else
    DIGITAL_OUT(Y2_ENABLE_PIN, enable.y);
   #endif
  #endif
  #ifdef Z_ENABLE_PIN
   #if Z_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(Z_ENABLE_PIN, enable.z);
   #else
    DIGITAL_OUT(Z_ENABLE_PIN, enable.z);
   #endif
  #endif
  #ifdef Z2_ENABLE_PIN
    DIGITAL_OUT(Z2_ENABLE_PIN, enable.z);
  #endif
  #ifdef A_ENABLE_PIN
    DIGITAL_OUT(A_ENABLE_PIN, enable.a);
  #endif
  #ifdef B_ENABLE_PIN
    DIGITAL_OUT(B_ENABLE_PIN, enable.b);
  #endif
  #ifdef C_ENABLE_PIN
    DIGITAL_OUT(C_ENABLE_PIN, enable.c);
  #endif
 #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    // Enable stepper drivers.
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);

    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0x00000000ULL);
//  timer_set_alarm_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 5000ULL);
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmhi.val = 0;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmlo.val = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up.
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_high = 0;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up.
#endif
    timer_start(STEP_TIMER_GROUP, STEP_TIMER_INDEX);
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.tn_alarm_en = TIMER_ALARM_EN;
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
#endif
}

// Sets up stepper driver interrupt timeout
IRAM_ATTR static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
// Limit min steps/s to about 2 (hal.f_step_timer @ 20MHz)
#if CONFIG_IDF_TARGET_ESP32S3
  #if ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmlo.val = cycles_per_tick < (1UL << 18) ? max(cycles_per_tick, t_min_period) : (1UL << 18) - 1UL;
  #else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmlo.val = cycles_per_tick < (1UL << 23) ? max(cycles_per_tick, t_min_period) : (1UL << 23) - 1UL;
  #endif
#else
  #if ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 18) ? max(cycles_per_tick, t_min_period) : (1UL << 18) - 1UL;
  #else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 23) ? max(cycles_per_tick, t_min_period) : (1UL << 23) - 1UL;
  #endif
#endif
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline IRAM_ATTR static void set_dir_outputs (axes_signals_t dir_out)
{
    dir_out.bits ^= settings.steppers.dir_invert.bits;

#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        uint_fast8_t idx, mask = 1;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) switch(idx) {

                case X_AXIS:
                    DIGITAL_OUT(X_DIRECTION_PIN, dir_out.x);
#if X_GANGED
                    DIGITAL_OUT(X2_DIRECTION_PIN, dir_out.x ^ settings.steppers.ganged_dir_invert.x);
#endif
                    break;

                case Y_AXIS:
                    DIGITAL_OUT(Y_DIRECTION_PIN, dir_out.y);
#if Y_GANGED
                    DIGITAL_OUT(Y2_DIRECTION_PIN, dir_out.y ^ settings.steppers.ganged_dir_invert.y);
#endif
                    break;

#ifdef Z_DIRECTION_PIN
                case Z_AXIS:
                    DIGITAL_OUT(Z_DIRECTION_PIN, dir_out.z);
  #if Z_GANGED
                    DIGITAL_OUT(Z2_DIRECTION_PIN, dir_out.z ^ settings.steppers.ganged_dir_invert.z);
  #endif
                    break;
#endif

#ifdef A_AXIS
                case A_AXIS:
                    DIGITAL_OUT(A_DIRECTION_PIN, dir_out.a);
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    DIGITAL_OUT(B_DIRECTION_PIN, dir_out.b);
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    DIGITAL_OUT(C_DIRECTION_PIN, dir_out.c);
                    break;
#endif
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

    DIGITAL_OUT(X_DIRECTION_PIN, dir_out.x);
#if X_GANGED
    DIGITAL_OUT(X2_DIRECTION_PIN, dir_out.x ^ settings.steppers.ganged_dir_invert.x);
#endif
    DIGITAL_OUT(Y_DIRECTION_PIN, dir_out.y);
#if Y_GANGED
    DIGITAL_OUT(Y2_DIRECTION_PIN, dir_out.y ^ settings.steppers.ganged_dir_invert.y);
#endif
#ifdef Z_DIRECTION_PIN
    DIGITAL_OUT(Z_DIRECTION_PIN, dir_out.z);
  #if Z_GANGED
    DIGITAL_OUT(Z2_DIRECTION_PIN, dir_out.z ^ settings.steppers.ganged_dir_invert.z);
  #endif
#endif
#ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PIN, dir_out.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PIN, dir_out.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PIN, dir_out.c);
#endif
#if STEP_INJECT_ENABLE
    }
#endif
}

#ifdef SQUARING_ENABLED

static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#endif // SQUARING_ENABLED

#if USE_I2S_OUT

static bool goIdlePending = false;
static uint32_t t_max_period;
static uint32_t i2s_step_length = I2S_OUT_USEC_PER_PULSE, i2s_delay_length = I2S_OUT_USEC_PER_PULSE, i2s_delay_samples = 1, i2s_step_samples = 1;
static bool laser_mode = false;
#if DRIVER_SPINDLE_ENABLE
static on_spindle_selected_ptr on_spindle_selected;
#endif

// Set stepper pulse output pins
inline __attribute__((always_inline)) IRAM_ATTR static void i2s_set_step_outputs (axes_signals_t step_outbits_1);

#if !CONFIG_IDF_TARGET_ESP32S3

IRAM_ATTR static void delay_us (uint32_t us)
{
#if CONFIG_IDF_TARGET_ESP32S3
    int32_t t = XTHAL_GET_CCOUNT() + hal.f_mcu * us;

    while((XTHAL_GET_CCOUNT() - t) < 0) {
        __asm__ __volatile__ ("nop");
    }
#else
    uint64_t start_time = esp_timer_get_time();

    while (esp_timer_get_time() - start_time < us) {
        __asm__ __volatile__ ("nop");
    }
#endif
}

#endif // !CONFIG_IDF_TARGET_ESP32S3

IRAM_ATTR static void I2SStepperCyclesPerTick (uint32_t cycles_per_tick)
{
    i2s_out_set_pulse_period((cycles_per_tick < (1UL << 18) ? max(cycles_per_tick, t_min_period) : (1UL << 18) - 1) / t_max_period);
}

// Sets stepper direction and pulse pins and starts a step pulse
// Called when in I2S stepping mode
IRAM_ATTR static void I2SStepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
        if(stepper->step_out.bits)
            i2s_out_push_sample(i2s_delay_samples);
    }

    if(stepper->step_out.bits) {
        i2s_set_step_outputs(stepper->step_out);
        i2s_out_push_sample(i2s_step_samples);
        i2s_set_step_outputs((axes_signals_t){0});
    }
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void I2SStepperWakeUp (void)
{
    // Enable stepper drivers.
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);
    i2s_out_set_stepping();
}

#ifdef SQUARING_ENABLED

// Set stepper pulse output pins
inline __attribute__((always_inline)) IRAM_ATTR static void i2s_set_step_outputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;
    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    DIGITAL_OUT(X_STEP_PIN, step_outbits_1.x);
    DIGITAL_OUT(Y_STEP_PIN, step_outbits_1.y);
#ifdef Z_STEP_PIN
    DIGITAL_OUT(Z_STEP_PIN, step_outbits_1.z);
#endif
#ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PIN, step_outbits_2.x);
#endif
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PIN, step_outbits_2.y);
#endif
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PIN, step_outbits_2.z);
#endif
#ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PIN, step_outbits_1.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PIN, step_outbits_1.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PIN, step_outbits_1.c);
#endif
}

#else // !SQUARING_ENABLED

// Set stepper pulse output pins
inline __attribute__((always_inline)) IRAM_ATTR static void i2s_set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;
    DIGITAL_OUT(X_STEP_PIN, step_outbits.x);
    DIGITAL_OUT(Y_STEP_PIN, step_outbits.y);
#ifdef Z_STEP_PIN
    DIGITAL_OUT(Z_STEP_PIN, step_outbits.z);
#endif
#ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PIN, step_outbits.x);
#endif
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PIN, step_outbits.y);
#endif
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PIN, step_outbits.z);
#endif
#ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PIN, step_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PIN, step_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PIN, step_outbits.c);
#endif
}

#endif // !SQUARING_ENABLED

void i2s_step_sink (void)
{
    //NOOP
}

void I2SReset (void)
{
    if(goIdlePending) {
        i2s_out_set_passthrough();
        i2s_out_delay();
//      i2s_out_reset();
        goIdlePending = false;
    }
}

IRAM_ATTR static void I2SStepperGoIdle (bool clear_signals)
{
    if(clear_signals) {
        i2s_set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
        i2s_out_reset();
    }

    if(!(goIdlePending = xPortInIsrContext())) {
        i2s_out_set_passthrough();
        i2s_out_delay();
    }
}

#else // RMT stepping

void initRMT (settings_t *settings)
{
    rmt_item32_t rmtItem[2];

    rmt_config_t rmtConfig = {
        .rmt_mode = RMT_MODE_TX,
        .clk_div = 20,
        .mem_block_num = 1,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.carrier_freq_hz = 0,
        .tx_config.carrier_duty_percent = 50,
        .tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW,
        .tx_config.idle_output_en = true
    };

    rmtItem[0].duration0 = (uint32_t)(settings->steppers.pulse_delay_microseconds > 0.0f ? 4.0f * settings->steppers.pulse_delay_microseconds : 1.0f);
    rmtItem[0].duration1 = (uint32_t)(4.0f * settings->steppers.pulse_microseconds);
    rmtItem[1].duration0 = 0;
    rmtItem[1].duration1 = 0;

//    hal.max_step_rate = 4000000UL / (rmtItem[0].duration0 + rmtItem[0].duration1); // + latency

    uint32_t channel;
    for(channel = 0; channel < (N_AXIS + N_GANGED); channel++) {

        rmtConfig.channel = channel;

        switch(channel) {
            case X_AXIS:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.x;
                rmtConfig.gpio_num = X_STEP_PIN;
                break;
            case Y_AXIS:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.y;
                rmtConfig.gpio_num = Y_STEP_PIN;
                break;
#ifdef Z_STEP_PIN
            case Z_AXIS:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.z;
                rmtConfig.gpio_num = Z_STEP_PIN;
                break;
#endif
#ifdef A_STEP_PIN
            case A_AXIS:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.a;
                rmtConfig.gpio_num = A_STEP_PIN;
                break;
#endif
#ifdef B_STEP_PIN
            case B_AXIS:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.b;
                rmtConfig.gpio_num = B_STEP_PIN;
                break;
#endif
#ifdef C_STEP_PIN
            case C_AXIS:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.c;
                rmtConfig.gpio_num = C_STEP_PIN;
                break;
#endif
#ifdef X2_STEP_PIN
            case X2_MOTOR:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.x;
                rmtConfig.gpio_num = X2_STEP_PIN;
                break;
#endif
#ifdef Y2_STEP_PIN
            case Y2_MOTOR:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.y;
                rmtConfig.gpio_num = Y2_STEP_PIN;
                break;
#endif
#ifdef Z2_STEP_PIN
            case Z2_MOTOR:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.z;
                rmtConfig.gpio_num = Z2_STEP_PIN;
                break;
#endif
        }
#ifndef Z_STEP_PIN
        if(channel == Z_AXIS)
            continue;
#endif
        rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
        rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
        rmt_config(&rmtConfig);
        rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], 2, 0);
    }
}

#ifdef SQUARING_ENABLED

// Set stepper pulse output pins
inline IRAM_ATTR static void set_step_outputs (axes_signals_t step_out1)
{
    axes_signals_t step_out2;

    step_out2.bits = step_out1.bits & motors_2.bits;
    step_out1.bits = step_out1.bits & motors_1.bits;

#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        uint_fast8_t idx, mask = 1;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) {

                if(step_out2.bits & mask) switch(idx) {
#if X_GANGED
                    case X_AXIS:
                        rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
                        rmt_ll_tx_start(&RMT, X2_MOTOR);
                        break;
#endif
#if Y_GANGED
                    case Y_AXIS:
                        rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
                        rmt_ll_tx_start(&RMT, Y2_MOTOR);
                        break;
#endif
#if Z_GANGED && defined(Z2_STEP_PIN)
                    case Z_AXIS:
    					rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
    					rmt_ll_tx_start(&RMT, Z2_MOTOR);
                        break;
#endif
                }

                if(step_out1.bits & mask) switch(idx) {

					case X_AXIS:
						rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
						rmt_ll_tx_start(&RMT, X_AXIS);
						break;

					case Y_AXIS:
						rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
						rmt_ll_tx_start(&RMT, Y_AXIS);
						break;
#ifdef Z_STEP_PIN
					case Z_AXIS:
						rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
						rmt_ll_tx_start(&RMT, Z_AXIS);
						break;
#endif
#ifdef A_AXIS
					case A_AXIS:
						rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
						rmt_ll_tx_start(&RMT, A_AXIS);
						break;
#endif
#ifdef B_AXIS
					case B_AXIS:
						rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
						rmt_ll_tx_start(&RMT, B_AXIS);
						break;
#endif
#ifdef C_AXIS
					case C_AXIS:
						rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
						rmt_ll_tx_start(&RMT, C_AXIS);
						break;
#endif
                }
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

    if(step_out1.x) {
        rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
        rmt_ll_tx_start(&RMT, X_AXIS);
    }
#ifdef X2_STEP_PIN
    if(step_out2.x) {
        rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
        rmt_ll_tx_start(&RMT, X2_MOTOR);
    }
#endif

    if(step_out1.y) {
        rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
        rmt_ll_tx_start(&RMT, Y_AXIS);
    }
#ifdef Y2_STEP_PIN
    if(step_out2.y) {
        rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
        rmt_ll_tx_start(&RMT, Y2_MOTOR);
    }
#endif

#ifdef Z_STEP_PIN
    if(step_out1.z) {
        rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
        rmt_ll_tx_start(&RMT, Z_AXIS);
    }
  #ifdef Z2_STEP_PIN
    if(step_out2.z) {
        rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
        rmt_ll_tx_start(&RMT, Z2_MOTOR);
    }
  #endif
#endif // Z_STEP_PIN

#ifdef A_STEP_PIN
    if(step_out1.a) {
        rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
        rmt_ll_tx_start(&RMT, A_AXIS);
    }
#endif

#ifdef B_STEP_PIN
    if(step_out1.b) {
        rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
        rmt_ll_tx_start(&RMT, B_AXIS);
    }
#endif

#ifdef C_STEP_PIN
    if(step_out1.c) {
        rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
        rmt_ll_tx_start(&RMT, C_AXIS);
    }
#endif
#if STEP_INJECT_ENABLE
}
#endif
}

#else // !SQUARING_ENABLED

// Set stepper pulse output pins
inline IRAM_ATTR static void set_step_outputs (axes_signals_t step_out)
{
#if STEP_INJECT_ENABLE
/*
	uint32_t mask;

	if(step_pulse.inject.out.z) {
		mask = 1 << (Z_AXIS * 3);
		if(RMT.int_raw.val & mask) {
			step_pulse.inject.out.z = Off;
			RMT.int_raw.val = mask;
		}
		step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;
	}
*/
    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        uint_fast8_t idx, mask = 1;

        for(idx = 0; idx < N_AXIS; idx++) {

            if((step_out.bits & mask) && !(axes.bits & mask)) switch(idx) {

                case X_AXIS:
                    rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
                    rmt_ll_tx_start(&RMT, X_AXIS);
#if X_GANGED
                    rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
                    rmt_ll_tx_start(&RMT, X2_MOTOR);
#endif
                    break;

                case Y_AXIS:
                    rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
                    rmt_ll_tx_start(&RMT, Y_AXIS);
#if Y_GANGED
                    rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
                    rmt_ll_tx_start(&RMT, Y2_MOTOR);
#endif
                    break;
#if Z_GANGED && defined(Z_STEP_PIN)
                case Z_AXIS:
					rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
					rmt_ll_tx_start(&RMT, Z_AXIS);
  #ifdef Z2_STEP_PIN
					rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
					rmt_ll_tx_start(&RMT, Z2_MOTOR);
  #endif
                    break;
#endif
#ifdef A_AXIS
                case A_AXIS:
                    rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
                    rmt_ll_tx_start(&RMT, A_AXIS);
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
                    rmt_ll_tx_start(&RMT, B_AXIS);
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
                    rmt_ll_tx_start(&RMT, C_AXIS);
                    break;
#endif
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

    if(step_out.x) {
        rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
        rmt_ll_tx_start(&RMT, X_AXIS);
#if X_GANGED
        rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
        rmt_ll_tx_start(&RMT, X2_MOTOR);
#endif
    }

    if(step_out.y) {
        rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
        rmt_ll_tx_start(&RMT, Y_AXIS);
#if Y_GANGED
        rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
        rmt_ll_tx_start(&RMT, Y2_MOTOR);
#endif
    }

#ifdef Z_STEP_PIN
    if(step_out.z) {
        rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
        rmt_ll_tx_start(&RMT, Z_AXIS);
  #if Z_GANGED
        rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
        rmt_ll_tx_start(&RMT, Z2_MOTOR);
  #endif
    }
#endif

#ifdef A_STEP_PIN
    if(step_out.a) {
        rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
        rmt_ll_tx_start(&RMT, A_AXIS);
    }
#endif
#ifdef B_STEP_PIN
    if(step_out.b) {
        rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
        rmt_ll_tx_start(&RMT, B_AXIS);
    }
#endif
#ifdef C_STEP_PIN
    if(step_out.c) {
        rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
        rmt_ll_tx_start(&RMT, C_AXIS);
    }
#endif
#if STEP_INJECT_ENABLE
}
#endif
}

#endif // !SQUARING_ENABLED

#if STEP_INJECT_ENABLE

static void stepperClaimMotor (uint_fast8_t axis_id, bool claim)
{
    if(claim)
        step_pulse.inject.claimed.mask |= ((1 << axis_id) & AXES_BITMASK);
    else {
        step_pulse.inject.claimed.mask &= ~(1 << axis_id);
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits; // For now...
    }
}

void stepperOutputStep (axes_signals_t step_out, axes_signals_t dir_out)
{
	if(step_out.bits) {

        uint_fast8_t idx = N_AXIS - 1, mask = 1 << (N_AXIS - 1);
        axes_signals_t axes = { .bits = step_out.bits };

        step_pulse.inject.out = step_out;
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits; // | step_out.bits; // It looks like the RMT enqueues step pulses...
        dir_out.bits ^= settings.steppers.dir_invert.bits;

        do {
            if(axes.bits & mask) {

                axes.bits ^= mask;

                switch(idx) {

					case X_AXIS:
						DIGITAL_OUT(X_DIRECTION_PIN, dir_out.x);
#if X_GANGED
						DIGITAL_OUT(X2_DIRECTION_PIN, dir_out.x ^ settings.steppers.ganged_dir_invert.x);
#endif
			            rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
			            rmt_ll_tx_start(&RMT, X_AXIS);
#if X_GANGED
			            rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
			            rmt_ll_tx_start(&RMT, X2_MOTOR);
#endif
						break;

					case Y_AXIS:
						DIGITAL_OUT(Y_DIRECTION_PIN, dir_out.y);
#if Y_GANGED
						DIGITAL_OUT(Y2_DIRECTION_PIN, dir_out.y ^ settings.steppers.ganged_dir_invert.y);
#endif
			            rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
			            rmt_ll_tx_start(&RMT, Y_AXIS);
#if Y_GANGED
			            rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
			            rmt_ll_tx_start(&RMT, Y2_MOTOR);
#endif
						break;

#ifdef Z_STEP_PIN
					case Z_AXIS:
						DIGITAL_OUT(Z_DIRECTION_PIN, dir_out.z);
  #if Z_GANGED
						DIGITAL_OUT(Z2_DIRECTION_PIN, dir_out.z ^ settings.steppers.ganged_dir_invert.z);
  #endif
						rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
						rmt_ll_tx_start(&RMT, Z_AXIS);
  #if Z_GANGED
						rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
						rmt_ll_tx_start(&RMT, Z2_MOTOR);
  #endif
						break;
#endif
#ifdef A_AXIS
					case A_AXIS:
						DIGITAL_OUT(A_DIRECTION_PIN, dir_out.a);
			            rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
			            rmt_ll_tx_start(&RMT, A_AXIS);
						break;
#endif
#ifdef B_AXIS
					case B_AXIS:
						DIGITAL_OUT(B_DIRECTION_PIN, dir_out.b);
			            rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
			            rmt_ll_tx_start(&RMT, B_AXIS);
						break;
#endif
#ifdef C_AXIS
					case C_AXIS:
						DIGITAL_OUT(C_DIRECTION_PIN, dir_out.c);
			            rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
			            rmt_ll_tx_start(&RMT, C_AXIS);
						break;
#endif
                }
            }
            idx--;
            mask >>= 1;
        } while(axes.bits);
    }
}

#endif // STEP_INJECT_ENABLE

#endif // RMT Stepping

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
  #if X_AUTO_SQUARE
        ganged.x = On;
  #endif

  #if Y_AUTO_SQUARE
        ganged.y = On;
  #endif

  #if Z_AUTO_SQUARE
        ganged.z = On;
  #endif
    } else {
  #if X_GANGED
        ganged.x = On;
  #endif

  #if Y_GANGED
        ganged.y = On;
  #endif

  #if Z_GANGED
        ganged.z = On;
  #endif
    }

    return ganged;
}

#endif // GANGING_ENABLED

// Sets stepper direction and pulse pins and starts a step pulse
// Called when in I2S passthrough mode

#if CONFIG_IDF_TARGET_ESP32S3

IRAM_ATTR static void stepperPulseStart (stepper_t *stepper)
{
#if USE_I2S_OUT
    static bool add_dir_delay = false;
#endif

    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
#if USE_I2S_OUT
        if(!(add_dir_delay = !!stepper->step_out.bits))
            i2s_out_commit(0, i2s_delay_samples);
#endif
    }

    if(stepper->step_out.bits) {
#if USE_I2S_OUT
        i2s_set_step_outputs(stepper->step_out);
        i2s_out_commit(i2s_step_samples, add_dir_delay ? i2s_delay_samples : 0);
        add_dir_delay = false;
#else
        set_step_outputs(stepper->step_out);
#endif
    }
}

#else

IRAM_ATTR static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
#if USE_I2S_OUT
        if(stepper->step_out.bits)
            delay_us(i2s_delay_length + 1);
#endif
    }

    if(stepper->step_out.bits) {
#if USE_I2S_OUT
        i2s_set_step_outputs(stepper->step_out);
        delay_us(i2s_step_length + 1);
        i2s_set_step_outputs((axes_signals_t){0});
#else
        set_step_outputs(stepper->step_out);
#endif
    }
}

#endif

// Disables stepper driver interrupt
IRAM_ATTR static void stepperGoIdle (bool clear_signals)
{
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.tn_en = 0;
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.enable = 0;
#endif
    if(clear_signals) {
#if USE_I2S_OUT
        i2s_set_step_outputs((axes_signals_t){0});
#else
        set_step_outputs((axes_signals_t){0});
#endif
        set_dir_outputs((axes_signals_t){0});
    }
}

#if USE_I2S_OUT

static void i2s_set_streaming_mode (bool stream)
{
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.tn_en = 0;
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.enable = 0;
#endif

    if(!stream && hal.stepper.wake_up == I2SStepperWakeUp && i2s_out_get_pulser_status() == STEPPING) {
       i2s_out_set_passthrough();
       i2s_out_delay();
    }

    if(stream) {
        if(hal.stepper.wake_up != I2SStepperWakeUp) {
            hal.stepper.wake_up = I2SStepperWakeUp;
            hal.stepper.go_idle = I2SStepperGoIdle;
            hal.stepper.cycles_per_tick = I2SStepperCyclesPerTick;
            hal.stepper.pulse_start = I2SStepperPulseStart;
            i2s_out_set_pulse_callback(hal.stepper.interrupt_callback);
        }
    } else if(hal.stepper.wake_up != stepperWakeUp) {
        hal.stepper.wake_up = stepperWakeUp;
        hal.stepper.go_idle = stepperGoIdle;
        hal.stepper.cycles_per_tick = stepperCyclesPerTick;
        hal.stepper.pulse_start = stepperPulseStart;
        i2s_out_set_pulse_callback(i2s_step_sink);
    }
}

#if DRIVER_SPINDLE_ENABLE

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    i2s_set_streaming_mode(!(laser_mode = spindle->cap.laser));

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

#endif

#endif // USE_I2S_OUT

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
#if USE_I2S_OUT
    i2s_set_streaming_mode(!(homing_cycle.mask || laser_mode));
#endif

    bool disable = !on;
    uint32_t i = limit_inputs.n_pins;
    axes_signals_t pin;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    if(i) do {
        i--;
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit_inputs.pins.inputs[i].id);
            disable = limit_inputs.pins.inputs[i].group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        gpio_set_intr_type(limit_inputs.pins.inputs[i].pin, on ? map_intr_type(limit_inputs.pins.inputs[i].mode.irq_mode) : GPIO_INTR_DISABLE);
        if(disable)
            gpio_intr_disable(limit_inputs.pins.inputs[i].pin);
        else
            gpio_intr_enable(limit_inputs.pins.inputs[i].pin);
    } while(i);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif

#ifdef X_LIMIT_PIN
    signals.min.x = DIGITAL_IN(X_LIMIT_PIN);
#endif
#ifdef Y_LIMIT_PIN
    signals.min.y = DIGITAL_IN(Y_LIMIT_PIN);
#endif
#ifdef Z_LIMIT_PIN
    signals.min.z = DIGITAL_IN(Z_LIMIT_PIN);
#endif
#ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(C_LIMIT_PIN);
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_PIN);
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_PIN);
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_PIN);
#endif

#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PIN_MAX);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PIN_MAX);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PIN_MAX);
#endif
#ifdef A_LIMIT_PIN_MAX
    signals.max.a = DIGITAL_IN(A_LIMIT_PIN_MAX);
#endif
#ifdef B_LIMIT_PIN_MAX
    signals.max.b = DIGITAL_IN(B_LIMIT_PIN_MAX);
#endif
#ifdef C_LIMIT_PIN_MAX
    signals.max.c = DIGITAL_IN(C_LIMIT_PIN_MAX);
#endif

    if(settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

static bool fpu_hack = false; // Needed to avoid awakening the stupid meditating guru that is overly sensitive to floats!

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

#ifdef RESET_PIN
#if ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_PIN);
#else
    signals.reset = DIGITAL_IN(RESET_PIN);
#endif
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PIN);
#endif

#ifdef SAFETY_DOOR_PIN
    if(safety_door && debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PIN);
#endif
#ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PIN);
#endif
#ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(MOTOR_WARNING_PIN);
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return aux_ctrl_scan_status(signals);
}

#if MPG_ENABLE == 1

static volatile bool mpg_event = false;

static void mpg_select (void)
{
    static volatile bool mux = false;

    if(!mux) {
        mux = true;
        stream_mpg_enable(!DIGITAL_IN(MPG_MODE_PIN));
        mux = false;
    }
}

static void mpg_enable (void *data)
{
    if(sys.mpg_mode == DIGITAL_IN(MPG_MODE_PIN))
        stream_mpg_enable(true);
}

#endif // MPG_ENABLE == 1

#if DRIVER_PROBES

// Returns the probe triggered pin state.
static bool probeGetState (void * input)
{
    return DIGITAL_IN(((input_signal_t *)input)->pin);
}

#endif // DRIVER_PROBES

IRAM_ATTR static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *aux_in;
    control_signals_t signals = {0};
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if((aux_in = aux_ctrl_in_get(port))) {

        switch(aux_in->function) {
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PIN) == 0);
                break;
#endif
#if MPG_ENABLE == 1
            case Input_MPGSelect:
                mpg_event = true;
                break;
#endif
            default:
                break;
        }
        signals.mask |= aux_in->signal.mask;
        if(aux_in->irq_mode == IRQ_Mode_Change) {
            uint_fast8_t idx = inputPinsCount;
            do {
                if(inputpin[--idx].pin == aux_in->gpio.pin) {
                    signals.deasserted = DIGITAL_IN(aux_in->gpio.pin) ^ inputpin[idx].mode.inverted;
                    break;
                }
            } while(idx);
        }
    }

    if(signals.mask) {
        if(!signals.deasserted) {
            fpu_hack = true;
            signals.mask |= systemGetState().mask;
        }
        hal.control.interrupt_callback(signals);
        fpu_hack = false;
    }

    if(xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin;

    if(aux_ctrl->input == NULL) {

        uint_fast8_t i = sizeof(inputpin) / sizeof(input_signal_t);

        do {
            --i;
            if(inputpin[i].group == PinGroup_AuxInput && inputpin[i].user_port == aux_ctrl->port)
                aux_ctrl->input = &inputpin[i];
        } while(i && aux_ctrl->input == NULL);
    }

    if((pin = aux_ctrl_claim_port(aux_ctrl))) {

        switch(aux_ctrl->function) {
#if PROBE_ENABLE
            case Input_Probe:
                hal.driver_cap.probe = probe_add(Probe_Default, aux_ctrl->port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;
#endif
#if PROBE2_ENABLE
            case Input_Probe2:
                hal.driver_cap.probe2 = probe_add(Probe_2, aux_ctrl->aux_port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;

#endif
#if TOOLSETTER_ENABLE
            case Input_Toolsetter:
                hal.driver_cap.toolsetter = probe_add(Probe_Toolsetter, aux_ctrl->aux_port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;
#endif
#if SAFETY_DOOR_ENABLE
            case Input_SafetyDoor:
                safety_door = (input_signal_t *)aux_ctrl->input;
                safety_door->mode.debounce = hal.driver_cap.software_debounce;
                break;
#endif
            default: break;
        }
    }

    return aux_ctrl->port != IOPORT_UNASSIGNED;
}

bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl)
{
    xbar_t *pin;

#ifdef USE_EXPANDERS
    if(aux_ctrl->gpio.port == (void *)EXPANDER_PORT) {
        if((iox_out[aux_ctrl->gpio.pin] = malloc(sizeof(xbar_t))))
            memcpy(iox_out[aux_ctrl->gpio.pin], aux_ctrl->output, sizeof(xbar_t));
        else
            aux_ctrl->port = IOPORT_UNASSIGNED;
    } else
#endif
        if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->port, NULL)))
            ioport_set_function(pin, aux_ctrl->function, NULL);
        else
            aux_ctrl->port = IOPORT_UNASSIGNED;

    return aux_ctrl->port != IOPORT_UNASSIGNED;
}

#if PPI_ENABLE && (DRIVER_SPINDLE_ENABLE || DRIVER_SPINDLE1_ENABLE)

#include "laser/ppi.h"

static void (*ppi_spindle_on)(void) = NULL;
static void (*ppi_spindle_off)(void) = NULL;
static spindle_ptrs_t *ppi_spindle = NULL;
static hal_timer_t ppi_timer;

static void spindlePulseOff (void *context)
{
    ppi_spindle_off();
}

static void spindlePulseOn (uint_fast16_t pulse_length)
{
    hal.timer.start(ppi_timer, pulse_length);
    ppi_spindle_on();
}

#endif // PPI_ENABLE

#if DRIVER_SPINDLE_ENABLE

// Static spindle (off, on cw & on ccw)


IRAM_ATTR inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
 #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
  #if SPINDLE_DIRECTION_PORT == EXPANDER_PORT
        EXPANDER_OUT(SPINDLE_DIRECTION_PIN, settings.pwm_spindle.invert.ccw);
  #else
        DIGITAL_OUT(SPINDLE_DIRECTION_PIN, settings.pwm_spindle.invert.ccw);
  #endif
    } else {
  #if SPINDLE_ENABLE_PORT == EXPANDER_PORT
        EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
  #else
        DIGITAL_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
  #endif
    }
 #elif SPINDLE_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
 #else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
 #endif
    spindle->context.pwm->flags.enable_out = Off;
#elif SPINDLE_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
#endif
}

IRAM_ATTR static void spindleOffBasic (spindle_ptrs_t *spindle)
{
    spindle_off(spindle);
}

IRAM_ATTR inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
 #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
  #if SPINDLE_DIRECTION_PORT == EXPANDER_PORT
        EXPANDER_OUT(SPINDLE_DIRECTION_PIN, !settings.pwm_spindle.invert.ccw);
  #else
        DIGITAL_OUT(SPINDLE_DIRECTION_PIN, !settings.pwm_spindle.invert.ccw);
  #endif
    } else {
  #if SPINDLE_ENABLE_PORT == EXPANDER_PORT
        EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
  #else
        DIGITAL_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
  #endif
    }
 #elif SPINDLE_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
 #else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
 #endif
    spindle->context.pwm->flags.enable_out = On;
#elif SPINDLE_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
#endif
}

IRAM_ATTR inline static void spindle_dir (bool ccw)
{
#if defined(SPINDLE_DIRECTION_PIN)
  #if SPINDLE_DIRECTION_PORT == EXPANDER_PORT
    EXPANDER_OUT(SPINDLE_DIRECTION_PIN, (ccw ^ settings.pwm_spindle.invert.ccw));
  #else
    DIGITAL_OUT(SPINDLE_DIRECTION_PIN, (ccw ^ settings.pwm_spindle.invert.ccw) ? 1 : 0);
  #endif
#endif
}

// Start or stop spindle
IRAM_ATTR static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Variable spindle control functions

static void pwm_off (spindle_ptrs_t *spindle)
{
#if PWM_RAMPED
        pwm_spindle.ramp.pwm_target = spindle->context.pwm->off_value;
        ledc_set_fade_step_and_start(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel, pwm_spindle.ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
        if(spindle->context.pwm->flags.always_on) {
            ledc_set_duty(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel, spindle->context.pwm->off_value);
            ledc_update_duty(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel);
        } else
            ledc_stop(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel, spindle->context.pwm->settings->invert.pwm);
#endif
}

// Sets spindle speed
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive) {
                ledc_set_duty(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel, spindle->context.pwm->pwm_overdrive);
                ledc_update_duty(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel);
            }
        } else
            pwm_off(spindle);

    } else {
#if PWM_RAMPED
         pwm_spindle.ramp.pwm_target = pwm_value;
         ledc_set_fade_step_and_start(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel, pwm_spindle.ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
         ledc_set_duty(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel, pwm_value);
         ledc_update_duty(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel);
#endif
        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle, variable version

IRAM_ATTR static void spindleOff (spindle_ptrs_t *spindle)
{
    if(spindle) {
        pwm_off(spindle);
        spindle_off(spindle);
    }
}

IRAM_ATTR static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle_off(spindle);
        pwm_off(spindle);
    } else {
#ifdef SPINDLE_DIRECTION_PIN
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle_off(spindle);
        else {
            spindle_on(spindle);
            spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if((spindle->cap.variable = !settings.pwm_spindle.flags.pwm_disable && settings.pwm_spindle.rpm_max > settings.pwm_spindle.rpm_min)) {

        spindle->esp32_off = spindleOff;
        spindle->set_state = spindleSetStateVariable;

        if(pwm_spindle.timer.freq_hz != (uint32_t)settings.pwm_spindle.pwm_freq) {
            pwm_spindle.timer.freq_hz = (uint32_t)settings.pwm_spindle.pwm_freq;
            if(pwm_spindle.timer.freq_hz <= 100) {
#if SOC_LEDC_TIMER_BIT_WIDE_NUM > 14
                if(pwm_spindle.timer.duty_resolution != LEDC_TIMER_16_BIT) {
                    pwm_spindle.timer.duty_resolution = LEDC_TIMER_16_BIT;
                    ledc_timer_config(&pwm_spindle.timer);
                }
#else
                if(pwm_spindle.timer.duty_resolution != LEDC_TIMER_14_BIT) {
                    pwm_spindle.timer.duty_resolution = LEDC_TIMER_14_BIT;
                    ledc_timer_config(&pwm_spindle.timer);
                }
#endif
            } else if(pwm_spindle.timer.duty_resolution != LEDC_TIMER_10_BIT) {
                pwm_spindle.timer.duty_resolution = LEDC_TIMER_10_BIT;
                ledc_timer_config(&pwm_spindle.timer);
            }
        }

        pwm_spindle.pwm_max_value = (1UL << pwm_spindle.timer.duty_resolution) - 1;
        pwm_spindle.spindle_pwm.offset = (settings.pwm_spindle.invert.pwm ? -1 : 1);
        spindle_precompute_pwm_values(spindle, &pwm_spindle.spindle_pwm, &settings.pwm_spindle, pwm_spindle.pwm_max_value * settings.pwm_spindle.pwm_freq);

        ledc_set_freq(pwm_spindle.timer.speed_mode, pwm_spindle.timer.timer_num, pwm_spindle.timer.freq_hz);

        if((pwm_spindle.spindle_pwm.flags.laser_mode_disable = settings.pwm_spindle.flags.laser_mode_disable)) {
#if PPI_ENABLE
            if(ppi_spindle == spindle)
                ppi_spindle = NULL;
            spindle->pulse_on = NULL;
        } else if(ppi_timer) {
            spindle->pulse_on = spindlePulseOn;
            ppi_spindle = spindle;
            ppi_spindle_on = spindle_on;
            ppi_spindle_off = spindle_off;
#endif
        }
    } else {
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
        if(spindle->context.pwm->flags.enable_out)
#endif
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->esp32_off = spindleOffBasic;
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &pwm_spindle.spindle_pwm : NULL);

#if USE_I2S_OUT
    if(spindle->id == spindle_get_default())
        i2s_set_streaming_mode(!(laser_mode = spindle->cap.laser));
#endif

    return true;
}

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    UNUSED(spindle);

#if SPINDLE_ENABLE_PORT == EXPANDER_PORT
    state.on = EXPANDER_IN(SPINDLE_ENABLE_PIN);
#else
    state.on = DIGITAL_IN(SPINDLE_ENABLE_PIN) != 0;
#endif

#if defined(SPINDLE_DIRECTION_PIN)
  #if SPINDLE_DIRECTION_PORT == EXPANDER_PORT
    state.ccw = EXPANDER_IN(SPINDLE_DIRECTION_PIN);
  #else
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PIN) != 0;
  #endif
#endif

    state.value ^= settings.pwm_spindle.invert.mask;

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    state.on |= spindle->param->state.on;
  #if PWM_RAMPED
    state.at_speed = ledc_get_duty(pwm_spindle.channel.speed_mode, pwm_spindle.channel.channel) == pwm_spindle.ramp.pwm_target;
  #endif
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE

// Static spindle (off, on cw & on ccw)
IRAM_ATTR inline static void spindle1_off (spindle_ptrs_t *spindle)
{
#ifdef SPINDLE1_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE1_DIRECTION_PIN, pwm_spindle1.config->cfg.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE1_ENABLE_PIN, pwm_spindle1.config->cfg.invert.on);
    }
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PIN, pwm_spindle1.config->cfg.invert.on);
#endif
}

IRAM_ATTR static void spindle1OffBasic (spindle_ptrs_t *spindle)
{
    spindle1_off(spindle);
}

IRAM_ATTR inline static void spindle1_on (spindle_ptrs_t *spindle)
{
#ifdef SPINDLE1_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE1_DIRECTION_PIN, !pwm_spindle1.config->cfg.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE1_ENABLE_PIN, !pwm_spindle1.config->cfg.invert.on);
    }
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PIN, !pwm_spindle1.config->cfg.invert.on);
#endif
}

IRAM_ATTR inline static void spindle1_dir (bool ccw)
{
#ifdef SPINDLE1_DIRECTION_PIN
    DIGITAL_OUT(SPINDLE1_DIRECTION_PIN, (ccw ^ pwm_spindle1.config->cfg.invert.ccw) ? 1 : 0);
#endif
}

// Start or stop spindle
IRAM_ATTR static void spindle1SetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle1_off(spindle);
    else {
        spindle1_dir(state.ccw);
        spindle1_on(spindle);
    }
}

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Variable spindle control functions

static void pwm1_off (spindle_ptrs_t *spindle)
{
#if PWM_RAMPED
        pwm_spindle1.ramp.pwm_target = spindle->context.pwm->off_value;
        ledc_set_fade_step_and_start(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel, pwm_spindle1.ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
        if(spindle->context.pwm->flags.always_on) {
            ledc_set_duty(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel, spindle->context.pwm->off_value);
            ledc_update_duty(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel);
        } else
            ledc_stop(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel, spindle->context.pwm->settings->invert.pwm ? 1 : 0);
#endif
}

// Sets spindle speed
IRAM_ATTR static void spindle1SetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive) {
                ledc_set_duty(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel, spindle->context.pwm->settings->invert.pwm ? pwm_spindle1.pwm_max_value - pwm_value : pwm_value);
                ledc_update_duty(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel);

            }
        } else
            pwm1_off(spindle);

    } else {
#if PWM_RAMPED
         pwm_spindle1.ramp.pwm_target = pwm_value;
         ledc_set_fade_step_and_start(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel, pwm_spindle1.ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
         ledc_set_duty(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel, spindle->context.pwm->settings->invert.pwm ? pwm_spindle1.pwm_max_value - pwm_value : pwm_value);
         ledc_update_duty(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel);
#endif
         if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
             spindle_on(spindle);
    }
}

static uint_fast16_t spindle1GetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle, variable version

IRAM_ATTR static void spindle1Off (spindle_ptrs_t *spindle)
{
    if(spindle) {
        spindle1_off(spindle);
        pwm1_off(spindle);
    }
}

IRAM_ATTR static void spindle1SetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle1_off(spindle);
        pwm1_off(spindle);
    } else {
#ifdef SPINDLE_DIRECTION_PIN
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle1_off(spindle);
        else {
            spindle1_on(spindle);
            spindle1SetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

bool spindle1Config (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if((spindle->cap.variable = !pwm_spindle1.config->cfg.flags.pwm_disable && pwm_spindle1.config->cfg.rpm_max > pwm_spindle1.config->cfg.rpm_min)) {

        spindle->esp32_off = spindle1Off;
        spindle->set_state = spindle1SetStateVariable;

        if(pwm_spindle1.timer.freq_hz != (uint32_t)pwm_spindle1.config->cfg.pwm_freq) {
            pwm_spindle1.timer.freq_hz = (uint32_t)pwm_spindle1.config->cfg.pwm_freq;
            if(pwm_spindle1.timer.freq_hz <= 100) {
#if SOC_LEDC_TIMER_BIT_WIDE_NUM > 14
                if(pwm_spindle1.timer.duty_resolution != LEDC_TIMER_16_BIT) {
                    pwm_spindle1.timer.duty_resolution = LEDC_TIMER_16_BIT;
                    ledc_timer_config(&pwm_spindle1.timer);
                }
#else
                if(pwm_spindle1.timer.duty_resolution != LEDC_TIMER_14_BIT) {
                    pwm_spindle1.timer.duty_resolution = LEDC_TIMER_14_BIT;
                    ledc_timer_config(&pwm_spindle1.timer);
                }
#endif
            } else if(pwm_spindle1.timer.duty_resolution != LEDC_TIMER_10_BIT) {
                pwm_spindle1.timer.duty_resolution = LEDC_TIMER_10_BIT;
                ledc_timer_config(&pwm_spindle1.timer);
            }
        }

        pwm_spindle1.pwm_max_value = (1UL << pwm_spindle1.timer.duty_resolution) - 1;
        pwm_spindle1.spindle_pwm.offset = (pwm_spindle1.config->cfg.invert.pwm ? -1 : 1);
        spindle_precompute_pwm_values(spindle, &pwm_spindle1.spindle_pwm, &pwm_spindle1.config->cfg, pwm_spindle1.pwm_max_value * pwm_spindle1.config->cfg.pwm_freq);

        ledc_set_freq(pwm_spindle1.timer.speed_mode, pwm_spindle1.timer.timer_num, pwm_spindle1.timer.freq_hz);

#if PPI_ENABLE
            if(ppi_spindle == NULL && ppi_timer) {
                spindle->pulse_on = spindlePulseOn;
                ppi_spindle = spindle;
                ppi_spindle_on = spindle1_on;
                ppi_spindle_off = spindle1_off;
            } else if(ppi_spindle != spindle)
                spindle->pulse_on = NULL;
#endif
    } else {
        if(spindle->context.pwm->flags.enable_out)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->esp32_off = spindle1OffBasic;
        spindle->set_state = spindle1SetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &pwm_spindle1.spindle_pwm : NULL);

#if USE_I2S_OUT
    if(spindle->id == spindle_get_default())
        i2s_set_streaming_mode(!(laser_mode = spindle->cap.laser));
#endif

    return true;
}

#endif // DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindle1GetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    UNUSED(spindle);

#if xIOEXPAND_ENABLE // TODO: read from expander?
    state.on = iopins.spindle_on;
    state.ccw = iopins.spindle_dir;
#else
 #if defined(SPINDLE1_ENABLE_PIN)
    state.on = DIGITAL_IN(SPINDLE1_ENABLE_PIN) != 0;
 #endif
 #if defined(SPINDLE_DIRECTION_PIN)
    state.ccw = DIGITAL_IN(SPINDLE1_DIRECTION_PIN) != 0;
 #endif
#endif
    state.value ^= pwm_spindle1.config->cfg.invert.mask;
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
    state.on |= spindle->param->state.on;
  #if PWM_RAMPED
    state.at_speed = ledc_get_duty(pwm_spindle1.channel.speed_mode, pwm_spindle1.channel.channel) == pwm_spindle1.ramp.pwm_target;
  #endif
#endif

    return state;
}

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

static void spindle1_settings_changed (spindle1_pwm_settings_t *settings)
{
    spindle1Config(spindle_get_hal(spindle1_id, SpindleHAL_Configured));
}

#endif

#endif // DRIVER_SPINDLE1_ENABLE

// Start/stop coolant (and mist if enabled)
IRAM_ATTR static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant.invert.mask;

#if defined(COOLANT_FLOOD_PIN)
  #if COOLANT_FLOOD_PORT == EXPANDER_PORT
    EXPANDER_OUT(COOLANT_FLOOD_PIN, mode.flood);
  #else
    DIGITAL_OUT(COOLANT_FLOOD_PIN, mode.flood);
  #endif
#endif

#if defined(COOLANT_MIST_PIN)
  #if COOLANT_MIST_PORT== EXPANDER_PORT
    EXPANDER_OUT(COOLANT_MIST_PIN, mode.mist);
  #else
    DIGITAL_OUT(COOLANT_MIST_PIN, mode.mist);
  #endif
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {settings.coolant.invert.mask};

#if defined(COOLANT_FLOOD_PIN)
  #if COOLANT_FLOOD_PORT == EXPANDER_PORT
    state.flood = EXPANDER_IN(COOLANT_FLOOD_PIN);
  #else
    state.flood = DIGITAL_IN(COOLANT_FLOOD_PIN);
  #endif
#endif

#if defined(COOLANT_MIST_PIN)
  #if COOLANT_MIST_PORT== EXPANDER_PORT
    state.mist = EXPANDER_IN(COOLANT_MIST_PIN);
  #else
    state.mist = DIGITAL_IN(COOLANT_MIST_PIN);
  #endif
#endif

    state.value ^= settings.coolant.invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
IRAM_ATTR static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    portENTER_CRITICAL(&mux);
    *ptr |= bits;
    portEXIT_CRITICAL(&mux);
}

IRAM_ATTR static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    portENTER_CRITICAL(&mux);
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    portEXIT_CRITICAL(&mux);
    return prev;
}

IRAM_ATTR static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    portENTER_CRITICAL(&mux);
    uint_fast16_t prev = *ptr;
    *ptr = value;
    portEXIT_CRITICAL(&mux);
    return prev;
}

static void enable_irq (void)
{
    portEXIT_CRITICAL(&mux);
}

static void disable_irq (void)
{
    portENTER_CRITICAL(&mux);
}

static inline uint64_t getElapsedMicros (void)
{
    return (uint64_t)esp_timer_get_time();
}

gpio_int_type_t map_intr_type (pin_irq_mode_t mode)
{
    switch(mode) {
        case IRQ_Mode_Rising:
            return GPIO_INTR_POSEDGE;
            break;
        case IRQ_Mode_Falling:
            return GPIO_INTR_NEGEDGE;
            break;
        case IRQ_Mode_Change:
            return GPIO_INTR_ANYEDGE;
            break;
        case IRQ_Mode_Edges:
            return GPIO_INTR_DISABLE;
            break;
        case IRQ_Mode_High:
            return GPIO_INTR_HIGH_LEVEL;
            break;
        case IRQ_Mode_Low:
            return GPIO_INTR_LOW_LEVEL;
            break;
        default:
            break;
    }

    return GPIO_INTR_DISABLE;
}

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    if(IOInitDone) {

#ifdef LED_PIN

    if(neopixel.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip.length0) {

        if(settings->rgb_strip.length0 == 0)
            settings->rgb_strip.length0 = hal.rgb0.num_devices;
        else
            hal.rgb0.num_devices = settings->rgb_strip.length0;

        if(neopixel.leds) {
            free(neopixel.leds);
            neopixel.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
            neopixel.num_bytes = hal.rgb0.num_devices * 3;
            if((neopixel.leds = calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb0.num_devices = 0;
        }

        neopixel.num_leds = hal.rgb0.num_devices;
    }

#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif

#if BLUETOOTH_ENABLE
        static bool bluetooth_ok = false;
        if(!bluetooth_ok)
            bluetooth_ok = bluetooth_start_local();
        // else report error?
#endif

#if WIFI_ENABLE

        static bool wifi_ok = false;

        if(!wifi_ok)
            wifi_ok = wifi_start();

        // TODO: start/stop services...
#endif

        /*********************
         * Step pulse config *
         *********************/

        t_min_period = (uint32_t)ceilf((hal.f_step_timer / 1000000.0f) * (settings->steppers.pulse_microseconds + STEP_PULSE_TOFF_MIN));

#if USE_I2S_OUT

        t_max_period = (uint32_t)(hal.f_step_timer / 1000000.0f);

        i2s_delay_length = (uint32_t)ceilf(settings->steppers.pulse_delay_microseconds);
        i2s_step_length = (uint32_t)ceilf(settings->steppers.pulse_microseconds);

        if(i2s_delay_length % I2S_OUT_USEC_PER_PULSE)
            i2s_delay_length += I2S_OUT_USEC_PER_PULSE - i2s_delay_length % I2S_OUT_USEC_PER_PULSE;

        i2s_delay_length = min(max(i2s_delay_length, I2S_OUT_USEC_PER_PULSE), I2S_OUT_USEC_PER_PULSE * 2);

        if(i2s_step_length % I2S_OUT_USEC_PER_PULSE)
            i2s_step_length += I2S_OUT_USEC_PER_PULSE - i2s_step_length % I2S_OUT_USEC_PER_PULSE;

        i2s_step_length = min(max(i2s_step_length, I2S_OUT_USEC_PER_PULSE), I2S_OUT_USEC_PER_PULSE * 4);

        i2s_delay_samples = i2s_delay_length / I2S_OUT_USEC_PER_PULSE;
        i2s_step_samples = i2s_step_length / I2S_OUT_USEC_PER_PULSE;

//        hal.max_step_rate = 250000UL / (i2s_delay_samples + i2s_step_samples);

#else
        initRMT(settings);
#endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        gpio_config_t config;
        input_signal_t *signal;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

        while(i--) {

            signal = &inputpin[i];

            if(signal->group == PinGroup_AuxInputAnalog)
                continue;

            if(!(signal->group == PinGroup_AuxInput || signal->group == PinGroup_MPG))
                signal->mode.irq_mode = IRQ_Mode_None;

            switch(signal->id) {

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_Down : PullMode_Up;
                    signal->mode.inverted = limit_fei.x;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_Down : PullMode_Up;
                    signal->mode.inverted = limit_fei.y;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_Down : PullMode_Up;
                    signal->mode.inverted = limit_fei.z;
                   break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                case Input_LimitA_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_Down : PullMode_Up;
                    signal->mode.inverted = limit_fei.a;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
                case Input_LimitB_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_Down : PullMode_Up;
                    signal->mode.inverted = limit_fei.b;
                    break;
#endif
#ifdef C_LIMIT_PIN
                case Input_LimitC:
                case Input_LimitC_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.c ? PullMode_Down : PullMode_Up;
                    signal->mode.inverted = limit_fei.c;
                    break;
#endif
#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
                case Input_SdCardDetect:
                    signal->mode.pull_mode = PullMode_Up;
                    signal->mode.irq_mode = IRQ_Mode_Change;
                    signal->mode.debounce = On;
                    break;
#endif
                default:
                    break;
            }

            switch(signal->group) {

                case PinGroup_Control:
                case PinGroup_Limit:
                case PinGroup_LimitMax:
                    signal->mode.debounce = hal.driver_cap.software_debounce;
                    signal->mode.irq_mode = signal->mode.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising;
#if ETHERNET_ENABLE
                    gpio_isr_handler_add(signal->pin, (signal->group & (PinGroup_Limit|PinGroup_LimitMax)) ? gpio_limit_isr : gpio_control_isr, signal);
#endif
                    break;

                case PinGroup_AuxInput:
#if ETHERNET_ENABLE
                    gpio_isr_handler_add(signal->pin, gpio_aux_isr, signal);
#endif
                    break;

                default:
                    break;
            }

            if(signal->pin != 0xFF) {

                gpio_intr_disable(signal->pin);

                config.pin_bit_mask = 1ULL << signal->pin;
                config.mode = GPIO_MODE_INPUT;
#if CONFIG_IDF_TARGET_ESP32S3
                config.pull_up_en = signal->mode.pull_mode == PullMode_Up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                config.pull_down_en = signal->mode.pull_mode == PullMode_Up ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
                // Early versions(?) has an internal pullup on 45 - https://github.com/espressif/esp-idf/issues/9731
#else
                config.pull_up_en = signal->mode.pull_mode == PullMode_Up && signal->pin < 34 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                config.pull_down_en = signal->mode.pull_mode == PullMode_Up || signal->pin >= 34 ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
#endif
                config.intr_type = (signal->group & (PinGroup_Limit|PinGroup_LimitMax)) ? GPIO_INTR_DISABLE : map_intr_type(signal->mode.irq_mode);

                signal->offset = signal->pin >= 32 ? 1 : 0;
                signal->mask = signal->offset == 0 ? (uint32_t)config.pin_bit_mask : (uint32_t)(config.pin_bit_mask >> 32);

    //            printf("IN %d - %d - %d : %x\n", signal->pin,  signal->offset, signal->mask, signal->invert);

                gpio_config(&config);
            }
        }

        hal.limits.enable(settings->limits.flags.hard_enabled, (axes_signals_t){0});
        aux_ctrl_irq_enable(settings, aux_irq_handler);
    }
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    uint32_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < inputPinsCount; i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        if(pin.group != PinGroup_AuxInputAnalog || inputpin[i].cap.analog)
            pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < outputPinsCount; i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin - (outputpin[i].pin < I2S_OUT_PIN_BASE ? 0 : I2S_OUT_PIN_BASE);
        pin.port = low_level || outputpin[i].pin < I2S_OUT_PIN_BASE ? NULL : "I2S";
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    pin.port = NULL;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description == NULL ? xbar_group_to_description(ppin->pin.group) : ppin->pin.description;

        pin_info(&pin, data);
    } while((ppin = ppin->next));
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

#if SDCARD_ENABLE

#if !SDCARD_SDIO
static bool bus_ok = false;
static spi_host_device_t spi_host;
#endif

static sdmmc_card_t *card = NULL;

static bool sdcard_unmount (FATFS **fs)
{
    if(card && esp_vfs_fat_sdcard_unmount("/sdcard", card) == ESP_OK) {
        card = NULL;
//        bus_ok = false;
//        spi_bus_free(SDSPI_DEFAULT_HOST);
    }

    return card == NULL;
}

#if SDCARD_SDIO
#include "driver/sdmmc_host.h"
#endif

static char *sdcard_mount (FATFS **fs)
{
    static FATFS *fatfs;

#if !SDCARD_SDIO

    if(!bus_ok)
        return NULL;
#endif

    if(card == NULL) {

        esp_err_t ret = ESP_FAIL;

        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        };

#if SDCARD_SDIO

        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

        slot_config.width = 4;
        host.slot = SDMMC_HOST_SLOT_0;
#ifdef SDMMC_FREQ_KHZ
        host.max_freq_khz = SDMMC_FREQ_KHZ;
#endif
        slot_config.clk = SD_CLK_PIN;
        slot_config.cmd = SD_CMD_PIN;
        slot_config.d0 = SD_D0_PIN;
        slot_config.d1 = SD_D1_PIN;
        slot_config.d2 = SD_D2_PIN;
        slot_config.d3 = SD_D3_PIN;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        if((ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card)) != ESP_OK)
            report_warning(ret == ESP_FAIL ? "Failed to mount filesystem" : "Failed to initialize SD card");

#else

        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

        sdspi_dev_handle_t dh;

        slot_config.gpio_cs = SD_CS_PIN;
        slot_config.host_id = spi_host;
        sdspi_host_init_device(&slot_config, &dh);

        host.slot = dh;
#ifdef SDMMC_FREQ_KHZ
        host.max_freq_khz = SDMMC_FREQ_KHZ;
#endif

        gpio_set_drive_capability(SD_CS_PIN, GPIO_DRIVE_CAP_3);

        if((ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card)) != ESP_OK)
            report_warning(ret == ESP_FAIL ? "Failed to mount filesystem" : "Failed to initialize SD card");

#endif // SDCARD_SDIO

    }

    if(card && fs) {

        if(fatfs == NULL)
            fatfs = malloc(sizeof(FATFS));

        *fs = fatfs && f_mount(fatfs, "", 1) == FR_OK ? fatfs : NULL;
    }

    return "";
}

#endif // SDCARD_ENABLE

#ifdef LED_PIN

//    https://github.com/adafruit/Adafruit_NeoPixel/blob/master/esp.c

#ifndef NEOPIXELS_NUM
#define NEOPIXELS_NUM 0
#endif

#if CONFIG_IDF_TARGET_ESP32S3
static rmt_config_t neo_config = RMT_DEFAULT_CONFIG_TX(LED_PIN, 3);
#else
static rmt_config_t neo_config = RMT_DEFAULT_CONFIG_TX(LED_PIN, 7);
#endif

#define WS2812_T0H_NS (450)
#define WS2812_T0L_NS (850)
#define WS2812_T1H_NS (800)
#define WS2812_T1L_NS (450)
/*
#define WS2812_T0H_NS (350)
#define WS2812_T0L_NS (1000)
#define WS2812_T1H_NS (1000)
#define WS2812_T1L_NS (350)

#define WS2812_T0H_NS (500)
#define WS2812_T0L_NS (2000)
#define WS2812_T1H_NS (1200)
#define WS2812_T1L_NS (1300)
*/

static uint32_t t0h_ticks = 0, t1h_ticks = 0, t0l_ticks = 0, t1l_ticks = 0;

static void IRAM_ATTR ws2812_rmt_adapter (const void *src, rmt_item32_t *dest, size_t src_size,
                                           size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    const rmt_item32_t bit0 = {{{ t0h_ticks, 1, t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ t1h_ticks, 1, t1l_ticks, 0 }}}; //Logical 1

    rgb_color_t color = {0};
    size_t size = 0, num = 0;
    uint8_t *psrc = (uint8_t *)src, bitmask;

    if(!(src == NULL || dest == NULL)) {
        while(size < src_size && num < wanted_num) {

            color.G = *psrc++;
            color.R = *psrc++;
            color.B = *psrc++;
            color = rgb_set_intensity(color, neopixel.intensity);

            bitmask = 0b10000000;
            do {
                dest->val = color.G & bitmask ? bit1.val : bit0.val;
                dest++;
            } while(bitmask >>= 1);

            bitmask = 0b10000000;
            do {
                dest->val = color.R & bitmask ? bit1.val : bit0.val;
                dest++;
            } while(bitmask >>= 1);

            bitmask = 0b10000000;
            do {
                dest->val = color.B & bitmask ? bit1.val : bit0.val;
                dest++;
            } while(bitmask >>= 1);

            num += 24;
            size += 3;
        }
    }

    *translated_size = size;
    *item_num = num;
}

void _neopixels_write (void)
{
    uint8_t *buf = neopixel.leds;
    size_t size = neopixel.num_bytes;

#if CONFIG_IDF_TARGET_ESP32S3
    if(buf) do {
        rmt_write_sample(neo_config.channel, buf, size > 3 ? 3 : size, true);
        buf += size > 3 ? 3 : size;
        size -= size > 3 ? 3 : size;
    } while(size);
#else
    if(buf) do {
        rmt_write_sample(neo_config.channel, buf, size > 6 ? 6 : size, true);
        buf += size > 6 ? 6 : size;
        size -= size > 6 ? 6 : size;
    } while(size);
#endif
}

void neopixels_write (void)
{
	if(neopixel.num_bytes > 1)
		_neopixels_write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        rgb_1bpp_assign(&neopixel.leds[device * 3], color, mask);

        if(neopixel.num_leds == 1)
        	_neopixels_write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

uint8_t neopixels_set_intensity (uint8_t value)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != value) {
        neopixel.intensity = value;
        _neopixels_write();
    }

    return prev;
}

#endif // LED_PIN

// Initializes MCU peripherals for grblHAL use
static bool driver_setup (settings_t *settings)
{

#if LITTLEFS_ENABLE
    fs_littlefs_mount(LITTLEFS_MOUNT_DIR, esp32_littlefs_hal());
#endif

    /******************
     *  Stepper init  *
     ******************/

    timer_config_t timerConfig = {
        .divider     = STEPPER_DRIVER_PRESCALER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .alarm_en    = TIMER_ALARM_EN,
        .intr_type   = TIMER_INTR_LEVEL,
        .auto_reload = true
    };

    timer_init(STEP_TIMER_GROUP, STEP_TIMER_INDEX, &timerConfig);
    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0ULL);
    timer_isr_register(STEP_TIMER_GROUP, STEP_TIMER_INDEX, stepper_driver_isr, 0, ESP_INTR_FLAG_LEVEL3|ESP_INTR_FLAG_IRAM, NULL);
    timer_enable_intr(STEP_TIMER_GROUP, STEP_TIMER_INDEX);

#if USE_I2S_OUT
    if(i2s_out_init()) {
#if CONFIG_IDF_TARGET_ESP32S3
        i2s_set_step_outputs((axes_signals_t){ .mask = AXES_BITMASK });
        i2s_set_step_mask();
#endif
        i2s_out_set_pulse_callback(hal.stepper.interrupt_callback);
    }
    // else report?
#endif

    /********************
     *  Output signals  *
     ********************/

    uint32_t idx;
    for(idx = 0; idx < (N_AXIS + N_GANGED); idx++) {
#ifndef Z_STEP_PIN
    	if(idx != Z_AXIS)
#endif
        rmt_set_source_clk(idx, RMT_BASECLK_APB);
    }

    uint64_t mask = 0;
    idx = sizeof(outputpin) / sizeof(output_signal_t);
    do {
        idx--;
        if(outputpin[idx].id == Output_SdCardCS || outputpin[idx].group == PinGroup_AuxOutputAnalog)
            continue;
#if USE_I2S_OUT
        else if(outputpin[idx].pin >= I2S_OUT_PIN_BASE)
            outputpin[idx].type = Pin_I2S;
#endif
        else if((outputpin[idx].type = outputpin[idx].group == PinGroup_StepperStep ? Pin_RMT : Pin_GPIO) == Pin_GPIO)
            mask |= (1ULL << outputpin[idx].pin);

    } while(idx);

#if !CONFIG_IDF_TARGET_ESP32S3
    if(mask & ((1ULL<<GPIO_NUM_34)|(1ULL<<GPIO_NUM_35)|(1ULL<<GPIO_NUM_36)|(1ULL<<GPIO_NUM_39))) {
        mask &= ~((1ULL<<GPIO_NUM_34)|(1ULL<<GPIO_NUM_35)|(1ULL<<GPIO_NUM_36)|(1ULL<< GPIO_NUM_39));
        task_run_on_startup(report_warning, "Input only pins configured for output!");
    }
#endif

    gpio_config_t gpioConfig = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&gpioConfig);

    idx = sizeof(outputpin) / sizeof(output_signal_t);
    do {
        idx--;
        if(outputpin[idx].group == PinGroup_MotorChipSelect || outputpin[idx].group == PinGroup_MotorUART) {
#if USE_I2S_OUT
            if(outputpin[idx].type == Pin_I2S)
                DIGITAL_OUT(outputpin[idx].pin, 1);
//            else
#endif
            DIGITAL_OUT(outputpin[idx].pin, 1);
        }
    } while(idx);

#if MPG_ENABLE == 1

    /************************
     *  MPG mode (pre)init  *
     ************************/

    // Set as output low (until boot is complete)
    gpioConfig.pin_bit_mask = (1ULL << MPG_MODE_PIN );
    gpio_config(&gpioConfig);
    DIGITAL_OUT(MPG_MODE_PIN , 0);

#endif

    /******************************************
     *  Control, limit & probe pins dir init  *
     ******************************************/

#if ETHERNET_ENABLE
    gpio_install_isr_service(0);
#else
    gpio_isr_register(gpio_isr, NULL, (int)ESP_INTR_FLAG_IRAM, NULL);
#endif

#if PWM_RAMPED
    ledc_fade_func_install(ESP_INTR_FLAG_IRAM);
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

    /******************
    *  Spindle init  *
    ******************/

    pwm_spindle.channel.speed_mode = pwm_spindle.timer.speed_mode;
    ledc_timer_config(&pwm_spindle.timer);
    ledc_channel_config(&pwm_spindle.channel);

    /**/

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

    /******************
    *  Spindle 1 init  *
    ******************/

    pwm_spindle1.channel.speed_mode = pwm_spindle1.timer.speed_mode;
    ledc_timer_config(&pwm_spindle1.timer);
    ledc_channel_config(&pwm_spindle1.channel);

#endif // DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

  // Set defaults

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});
    hal.stepper.go_idle(true);

#if USE_I2S_OUT && defined(SPINDLE_PWM_PIN)
    on_spindle_selected = grbl.on_spindle_selected;
    grbl.on_spindle_selected = onSpindleSelected;
#endif

#if SDCARD_ENABLE

    sdcard_events_t *card = sdcard_init();
    card->on_mount = sdcard_mount;
    card->on_unmount = sdcard_unmount;

#ifdef SD_DETECT_PIN
    if(!DIGITAL_IN(SD_DETECT_PIN))
        sdcard_detect(true);
#else
    sdcard_detect(true);
#endif

#endif // SDCARD_ENABLE

#if ETHERNET_ENABLE
    enet_start();
#endif

    return IOInitDone;
}

static bool set_rtc_time (struct tm *time)
{
    const struct timezone tz = {
        .tz_minuteswest = 0,
        .tz_dsttime = 0
    };
    struct timeval t;
    t.tv_sec = mktime(time);
    t.tv_usec = 0;

    return (hal.driver_cap.rtc_set = settimeofday(&t, &tz) == 0);
}

static bool get_rtc_time (struct tm *dt)
{
    bool ok = false;

    if(hal.driver_cap.rtc_set) {
        time_t now;
        if((ok = time(&now) != (time_t)-1))
            localtime_r(&now, dt);
    }

    return ok;
}

// Keep idle task alive
static void wdt_tickler (sys_state_t state)
{
    static uint32_t ms = 0;

#if MPG_ENABLE == 1
    if(mpg_event) {
        mpg_event = false;
        mpg_select();
    }
#endif

    if(xTaskGetTickCountFromISR() - ms > 250) {
        ms = xTaskGetTickCountFromISR();
        vTaskDelay(1);
    } else
        on_execute_realtime(state);
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: grblHAL is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for grblHAL to be able to configure itself and report any errors
    rtc_cpu_freq_config_t cpu;

    rtc_clk_cpu_freq_get_config(&cpu);

#if CONFIG_IDF_TARGET_ESP32S3
    hal.info = "ESP32-S3";
#else
    hal.info = "ESP32";
#endif
    hal.driver_version = "251015";
    hal.driver_url = GRBL_URL "/ESP32";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif
    hal.driver_options = IDF_VER;
    hal.driver_setup = driver_setup;
    hal.f_mcu = cpu.freq_mhz;
    hal.f_step_timer = rtc_clk_apb_freq_get() / STEPPER_DRIVER_PRESCALER; // 20 MHz

    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.get_free_mem = esp_get_free_heap_size;
    hal.delay_ms = driver_delay_ms;

    hal.timer.claim = timerClaim;
    hal.timer.configure = timerCfg;
    hal.timer.start = timerStart;
    hal.timer.stop = timerStop;

    hal.settings_changed = settings_changed;

#if USE_I2S_OUT
    hal.step_us_min = I2S_OUT_USEC_PER_PULSE;
    hal.driver_reset = I2SReset;
    hal.stepper.wake_up = I2SStepperWakeUp;
    hal.stepper.go_idle = I2SStepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = I2SStepperCyclesPerTick;
    hal.stepper.pulse_start = I2SStepperPulseStart;
#else
    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#endif
#if STEP_INJECT_ENABLE
    hal.stepper.output_step = stepperOutputStep;
    hal.stepper.claim_motor = stepperClaimMotor;
#endif
    hal.stepper.motor_iterator = motor_iterator;
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.control.get_state = systemGetState;

    hal.reboot = esp_restart;
    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_micros = getElapsedMicros;
    hal.get_elapsed_ticks = xTaskGetTickCountFromISR;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

    hal.driver_cap.rtc = On;
    hal.rtc.get_datetime = get_rtc_time;
    hal.rtc.set_datetime = set_rtc_time;

    serialRegisterStreams();

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = wdt_tickler;

#if USB_SERIAL_CDC
    stream_connect(usb_serialInit());
#else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else
    static const esp_partition_t *partition;

    if((partition = nvsInit())) {
        hal.nvs.type = NVS_Flash;
        hal.nvs.memcpy_from_flash = nvsRead;
        hal.nvs.memcpy_to_flash = nvsWrite;
        hal.nvs.size_max = partition->size;
    } else
        hal.nvs.type = NVS_None;
#endif

#if NVSDATA_BUFFER_ENABLE
    if(hal.nvs.type != NVS_None)
    	nvs_buffer_alloc(); // Reallocate memory block for NVS buffer
#endif

#if DRIVER_SPINDLE_ENABLE

  #if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_PWM0,
#else
        .ref_id = SPINDLE_PWM0_NODIR,
#endif
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
        .esp32_off = spindleOff,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On,
  #endif
  #if PWM_RAMPED
            .at_speed = On
  #endif
        }
    };

  #if PPI_ENABLE

    static timer_cfg_t cfg = {
        .single_shot = On,
        .timeout_callback = spindlePulseOff
    };

    if((ppi_timer = hal.timer.claim((timer_cap_t){ .periodic = Off }, 1000))) {
        hal.timer.configure(ppi_timer, &cfg);
        ppi_init();
    }

  #endif

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_ONOFF0_DIR,
  #else
        .ref_id = SPINDLE_ONOFF0,
  #endif
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .esp32_off = spindleOffBasic,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE

 #if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
    static const spindle_ptrs_t spindle1 = {
        .type = SpindleType_PWM,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_PWM1,
  #else
        .ref_id = SPINDLE_PWM1_NODIR,
  #endif
        .config = spindle1Config,
        .set_state = spindle1SetStateVariable,
        .get_state = spindle1GetState,
        .get_pwm = spindle1GetPWM,
        .update_pwm = spindle1SetSpeed,
        .esp32_off = spindle1Off,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
            .direction = On,
  #endif
  #if PWM_RAMPED
            .at_speed = On
  #endif
        }
    };

  #if PPI_ENABLE

    static timer_cfg_t cfg1 = {
        .single_shot = On,
        .timeout_callback = spindlePulseOff
    };

    if((ppi_timer = hal.timer.claim((timer_cap_t){ .periodic = Off }, 1000))) {
        hal.timer.configure(ppi_timer, &cfg1);
        ppi_init();
    }

  #endif

    if((pwm_spindle1.config = spindle1_settings_add(false))) {
        if((spindle1_id = spindle_register(&spindle1, DRIVER_SPINDLE1_NAME)) != -1)
            spindle1_settings_register(spindle1.cap, spindle1_settings_changed);
        else
            task_run_on_startup(report_warning, "PWM2 spindle failed to initialize!");
    }

 #else

    static const spindle_ptrs_t spindle1 = {
        .type = SpindleType_Basic,
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_ONOFF1_DIR,
#else
        .ref_id = SPINDLE_ONOFF1,
#endif
        .set_state = spindle1SetState,
        .get_state = spindle1GetState,
        .esp32_off = spindle1OffBasic,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    if((pwm_spindle1.config = spindle1_settings_add(false)) && (spindle1_id = spindle_register(&spindle1, DRIVER_SPINDLE1_NAME)) != -1)
        spindle1_settings_register(spindle1.cap, NULL);

 #endif

#endif // DRIVER_SPINDLE_ENABLE

  // driver capabilities, used for announcing and negotiating (with the core) driver functionality

#ifdef COOLANT_FLOOD_PIN
    hal.coolant_cap.flood = On;
#endif
#ifdef COOLANT_MIST_PIN
    hal.coolant_cap.mist = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
#if ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
    hal.signals_cap.reset = Off;
#endif
    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();

    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0},
                            aux_analog_in = {0}, aux_analog_out = {0};

    uint32_t i;
    input_signal_t *input;

    for(i = 0 ; i < inputPinsCount; i++) {
        input = &inputpin[i];
        input->mode.input = input->cap.input = On;
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->user_port = aux_inputs.n_pins++;
            input->id = (pin_function_t)(Input_Aux0 + input->user_port);
            input->cap.debounce = On;
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.irq_mode = IRQ_Mode_Edges;
            input->mode.pull_mode = PullMode_Up;

            aux_ctrl_t *aux_remap;
            if((aux_remap = aux_ctrl_remap_explicit((aux_gpio_t){ .pin = input->pin }, input->user_port, input))) {
                if(aux_remap->function == Input_Probe)
                    aux_remap->irq_mode = IRQ_Mode_Change;
            }

        } else if(input->group == PinGroup_Limit || input->group == PinGroup_LimitMax) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
            input->mode.debounce = hal.driver_cap.software_debounce;
        } else if(input->group == PinGroup_AuxInputAnalog) {
            if(aux_analog_in.pins.inputs == NULL)
                aux_analog_in.pins.inputs = input;
            input->mode.analog = input->cap.analog = On;
            input->id = (pin_function_t)(Input_Analog_Aux0 + aux_analog_in.n_pins++);
        } else if(input->group == PinGroup_Control)
            input->mode.debounce = hal.driver_cap.software_debounce;
    }

    output_signal_t *output;

    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        output->mode.output = On;
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins);

            aux_out_remap_explicit((aux_gpio_t){ .pin = output->pin }, aux_outputs.n_pins, output);

            aux_outputs.n_pins++;
        } else if(output->group == PinGroup_AuxOutputAnalog) {
            if(aux_analog_out.pins.outputs == NULL)
                aux_analog_out.pins.outputs = output;
            output->mode.analog = On;
            output->id = (pin_function_t)(Output_Analog_Aux0 + aux_analog_out.n_pins++);
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);
    if(aux_analog_in.n_pins || aux_analog_out.n_pins)
        ioports_init_analog(&aux_analog_in, &aux_analog_out);

    io_expanders_init();
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);
    aux_ctrl_claim_out_ports(aux_out_claim_explicit, NULL);

#ifdef LED_PIN

    neo_config.clk_div = 2;

    rmt_config(&neo_config);
    rmt_driver_install(neo_config.channel, 0, 0);

    uint32_t counter_clk_hz = 0;

    rmt_get_counter_clock(neo_config.channel, &counter_clk_hz);

    // NS to tick converter
    float ratio = (float)counter_clk_hz / 1e9;

    t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);

    // Initialize automatic timing translator
    rmt_translator_init(neo_config.channel, ws2812_rmt_adapter);

    hal.rgb0.out = neopixel_out;
    hal.rgb0.out_masked = neopixel_out_masked;
    hal.rgb0.set_intensity = neopixels_set_intensity;
    hal.rgb0.write = neopixels_write;
    hal.rgb0.num_devices = NEOPIXELS_NUM;
    hal.rgb0.flags = (rgb_properties_t){ .is_blocking = On, .is_strip = On };
    hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

    const periph_pin_t neopixels = {
        .function = Output_LED_Adressable,
        .group = PinGroup_LED,
        .pin = LED_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    hal.periph_port.register_pin(&neopixels);

#endif

#if TRINAMIC_SPI_ENABLE
    extern void tmc_spi_init (void);
    tmc_spi_init();
#elif TRINAMIC_UART_ENABLE
    extern void tmc_uart_init (void);
    tmc_uart_init();
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if WIFI_ENABLE
    wifi_init();
#endif

#if ETHERNET_ENABLE
    enet_init();
#endif

#if BLUETOOTH_ENABLE
    bluetooth_init_local();
#endif

#if SDCARD_ENABLE && !SDCARD_SDIO
    bus_ok = spi_bus_init(&spi_host);
#endif

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, "MPG"), MPG_SHARE_TX, NULL);
    if(hal.driver_cap.mpg_mode)
        task_run_on_startup(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, "MPG"), MPG_SHARE_TX, stream_mpg_check_enable);
#endif

    // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 10;
}

void pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#if SAFETY_DOOR_ENABLE
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
          DIGITAL_IN(input->pin) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1))
        switch(input->group) {

            case PinGroup_Limit:
            case PinGroup_LimitMax:
                hal.limits.interrupt_callback(limitsGetState());
                break;

            case PinGroup_Control:
                hal.control.interrupt_callback(systemGetState());
                break;

            case PinGroup_AuxInput:
                ioports_event(input);
                break;

#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
            case PinGroup_SdCard:
                sdcard_detect(!DIGITAL_IN(SD_DETECT_PIN));
                break;
#endif

            default:
                break;
    }

    gpio_intr_enable(input->pin);
}

/* interrupt handlers */

// Main stepper driver
IRAM_ATTR static void stepper_driver_isr (void *arg)
{
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG0.int_clr_timers.t0_int_clr = 1;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.tn_alarm_en = TIMER_ALARM_EN;
#else
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
#endif
    hal.stepper.interrupt_callback();
}

#if ETHERNET_ENABLE

IRAM_ATTR static void gpio_limit_isr (void *signal)
{
    if(((input_signal_t *)signal)->cap.debounce)
        task_add_delayed(pin_debounce, (input_signal_t *)signal, 40);
    else
        hal.limits.interrupt_callback(limitsGetState());
}

IRAM_ATTR static void gpio_control_isr (void *signal)
{
    if(((input_signal_t *)signal)->cap.debounce)
        task_add_delayed(pin_debounce, (input_signal_t *)signal, 40);
    else
        hal.control.interrupt_callback(systemGetState());
}

IRAM_ATTR static void gpio_aux_isr (void *signal)
{
    ioports_event((input_signal_t *)signal);
}

#if MPG_ENABLE == 1

IRAM_ATTR static void gpio_mpg_isr (void *signal)
{
    mpg_event = true;
}

#endif

#if I2C_STROBE_ENABLE
IRAM_ATTR static void gpio_i2c_strobe_isr (void *signal)
{
    if(i2c_strobe.callback)
        i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PIN));
}
#endif

#else // ETHERNET_ENABLE

//GPIO IRQ process
IRAM_ATTR static void gpio_isr (void *arg)
{
    uint32_t grp = 0, intr_status[2];

    gpio_ll_get_intr_status(&GPIO, GRBLHAL_TASK_CORE, &intr_status[0]);         // get interrupt status for GPIO0-31
    gpio_ll_get_intr_status_high(&GPIO, GRBLHAL_TASK_CORE, &intr_status[1]);    // get interrupt status for GPIO32-39
    gpio_ll_clear_intr_status(&GPIO, intr_status[0]);                           // clear intr for gpio0-gpio31
    gpio_ll_clear_intr_status_high(&GPIO, intr_status[1]);                      // clear intr for gpio32-39

    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
    while(i--) {
        if(intr_status[inputpin[i].offset] & inputpin[i].mask) {
            if(inputpin[i].mode.debounce && task_add_delayed(pin_debounce, &inputpin[i], 40)) {
#if SAFETY_DOOR_ENABLE
                if(inputpin[i].id == Input_SafetyDoor)
                    debounce.safety_door = On;
#endif
                gpio_intr_disable(inputpin[i].pin);
            } else if(inputpin[i].group & PinGroup_AuxInput)
                ioports_event(&inputpin[i]);
            else
                grp |= inputpin[i].group;
        }
    };

    if(grp & (PinGroup_Limit|PinGroup_LimitMax))
        hal.limits.interrupt_callback(limitsGetState());

    if(grp & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());
}

#endif
