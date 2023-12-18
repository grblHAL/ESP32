/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2018-2023 Terje Io

  Some parts
   Copyright (c) 2011-2015 Sungeun K. Jeon
   Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "./driver.h"
#include "uart_serial.h"
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

//#include "grbl_esp32_if/grbl_esp32_if.h"

#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/motor_pins.h"
#include "grbl/machine_limits.h"

#if GRBL_ESP32S3
#include "usb_serial.h"
#endif

#if USE_I2S_OUT
#include "i2s_out.h"
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
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

#if LITTLEFS_ENABLE
#include "littlefs_hal.h"
#include "sdcard/fs_littlefs.h"
#endif

#if KEYPAD_ENABLE == 2
#include "keypad/keypad.h"
#endif

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if I2C_ENABLE
#include "i2c.h"
#endif

#if DRIVER_SPINDLE_ENABLE

static spindle_id_t spindle_id = -1;

#if DRIVER_SPINDLE_PWM_ENABLE

#define pwm(s) ((spindle_pwm_t *)s->context)

static uint32_t pwm_max_value;
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;

static ledc_timer_config_t spindle_pwm_timer = {
#if GRBL_ESP32S3
    .speed_mode = LEDC_LOW_SPEED_MODE,
#else
    .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 5000
};

static ledc_channel_config_t spindle_pwm_channel = {
    .gpio_num = SPINDLE_PWM_PIN,
#if GRBL_ESP32S3
    .speed_mode = LEDC_SPEED_MODE_MAX,
#else
    .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
    .hpoint = 0
};

#endif // DRIVER_SPINDLE_PWM_ENABLE
#endif // DRIVER_SPINDLE_ENABLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// prescale step counter to 20Mhz
#define STEPPER_DRIVER_PRESCALER 4

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

static pwm_ramp_t pwm_ramp;
#endif

static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
#ifdef RESET_PIN
    { .id = Input_Reset,        .pin = RESET_PIN,         .group = PinGroup_Control },
#endif
#ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,     .pin = FEED_HOLD_PIN,     .group = PinGroup_Control },
#endif
#ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,   .pin = CYCLE_START_PIN,   .group = PinGroup_Control },
#endif
#ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,   .pin = SAFETY_DOOR_PIN,   .group = PinGroup_Control },
#endif
#ifdef PROBE_PIN
    { .id = Input_Probe,        .pin = PROBE_PIN,         .group = PinGroup_Probe },
#endif
#ifdef X_LIMIT_PIN
    { .id = Input_LimitX,       .pin = X_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,     .pin = X2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef Y_LIMIT_PIN
    { .id = Input_LimitY,       .pin = Y_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,     .pin = Y2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef Z_LIMIT_PIN
    { .id = Input_LimitZ,       .pin = Z_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,     .pin = Z2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef A_LIMIT_PIN
    { .id = Input_LimitA,       .pin = A_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,       .pin = B_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,       .pin = C_LIMIT_PIN,       .group = PinGroup_Limit },
#endif
#if MPG_MODE == 1
    { .id = Input_ModeSelect,   .pin = MPG_ENABLE_PIN,    .group = PinGroup_MPG },
#endif
#ifdef I2C_STROBE_PIN
    { .id = Input_I2CStrobe,    .pin = I2C_STROBE_PIN,    .group = PinGroup_Keypad },
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
};

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
#if defined(STEPPERS_ENABLE_PIN) && STEPPERS_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnable,  .pin = STEPPERS_ENABLE_PIN,   .group = PinGroup_StepperEnable },
#endif
#if defined(X_ENABLE_PIN) && X_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableX, .pin = X_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableY, .pin = Y_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableZ, .pin = Z_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(A_ENABLE_PIN) && A_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableA, .pin = A_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(B_ENABLE_PIN) && B_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableB, .pin = B_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(C_ENABLE_PIN) && C_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableC, .pin = C_ENABLE_PIN,          .group = PinGroup_StepperEnable },
#endif
#if defined(X2_ENABLE_PIN) && X2_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableX, .pin = X2_ENABLE_PIN,         .group = PinGroup_StepperEnable },
#endif
#if defined(Y2_ENABLE_PIN) && Y2_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableY, .pin = Y2_ENABLE_PIN,         .group = PinGroup_StepperEnable },
#endif
#if defined(Z2_ENABLE_PIN) && Z2_ENABLE_PIN != IOEXPAND
    { .id = Output_StepperEnableZ, .pin = Z2_ENABLE_PIN,         .group = PinGroup_StepperEnable },
#endif
#if DRIVER_SPINDLE_ENABLE
  #if defined(SPINDLE_ENABLE_PIN) && SPINDLE_ENABLE_PIN != IOEXPAND
    { .id = Output_SpindleOn,      .pin = SPINDLE_ENABLE_PIN,    .group = PinGroup_SpindleControl },
  #endif
  #if defined(SPINDLE_DIRECTION_PIN) && SPINDLE_DIRECTION_PIN != IOEXPAND
    { .id = Output_SpindleDir,     .pin = SPINDLE_DIRECTION_PIN, .group = PinGroup_SpindleControl },
  #endif
#endif // DRIVER_SPINDLE_ENABLE
#if defined(COOLANT_FLOOD_PIN) && COOLANT_FLOOD_PIN != IOEXPAND
    { .id = Output_CoolantFlood,   .pin = COOLANT_FLOOD_PIN,     .group = PinGroup_Coolant },
#endif
#if defined(COOLANT_MIST_PIN) && COOLANT_MIST_PIN != IOEXPAND
    { .id = Output_CoolantMist,    .pin = COOLANT_MIST_PIN,      .group = PinGroup_Coolant },
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
#ifdef PIN_NUM_CS
    { .id = Output_SdCardCS,          .pin = PIN_NUM_CS,        .group = PinGroup_SdCard },
#endif
#ifdef MODBUS_DIRECTION_PIN
    { .id = Output_Aux0,           .pin = MODBUS_DIRECTION_PIN,  .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT0_PIN
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
    { .id = Output_Aux3,           .pin = AUXOUTPUT6_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PIN
    { .id = Output_Aux7,           .pin = AUXOUTPUT7_PIN,        .group = PinGroup_AuxOutput }
#endif
};

static bool IOInitDone = false, rtc_started = false;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED, debounce_mux = portMUX_INITIALIZER_UNLOCKED;
#if PROBE_ENABLE
static probe_state_t probe = {
    .connected = On
};
#endif

#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

#if USE_I2S_OUT
static bool goIdlePending = false;
static uint32_t i2s_step_length = I2S_OUT_USEC_PER_PULSE, i2s_delay_length = I2S_OUT_USEC_PER_PULSE, i2s_delay_samples = 1, i2s_step_samples = 1;
static bool laser_mode = false;
#if DRIVER_SPINDLE_ENABLE
static on_spindle_selected_ptr on_spindle_selected;
#endif
#endif // USE_I2S_OUT

#if IOEXPAND_ENABLE
static ioexpand_t iopins = {0};
#endif

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
#if MPG_MODE == 1
static void gpio_mpg_isr (void *signal);
#endif
#if I2C_STROBE_ENABLE
static void gpio_i2c_strobe_isr (void *signal);
#endif
#else
static void gpio_isr (void *arg);
#endif
static void stepper_driver_isr (void *arg);

static TimerHandle_t xDelayTimer = NULL, debounceTimer = NULL;

#if USE_I2S_OUT

// Set stepper pulse output pins
inline __attribute__((always_inline)) IRAM_ATTR static void i2s_set_step_outputs (axes_signals_t step_outbits_1);

#else

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

#endif

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
        xDelayTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(ms), pdFALSE, callback, vTimerCallback);
        xTimerStartFromISR(xDelayTimer, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
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

#ifdef DEBUGOUT
static void debug_out (bool enable)
{
    gpio_set_level(STEPPERS_ENABLE_PIN, enable);
}
#endif

// Enable/disable steppers
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;

#if !TRINAMIC_MOTOR_ENABLE
  #if IOEXPAND_ENABLE // TODO: read from expander?
    iopins.stepper_enable_x = enable.x;
    iopins.stepper_enable_y = enable.y;
    iopins.stepper_enable_z = enable.z;
    ioexpand_out(iopins);
  #else
    #if defined(STEPPERS_ENABLE_PIN)
      #if STEPPERS_ENABLE_PIN >= I2S_OUT_PIN_BASE
        DIGITAL_OUT(STEPPERS_ENABLE_PIN, enable.x);
      #else
        gpio_set_level(STEPPERS_ENABLE_PIN, enable.x);
      #endif
    #else
      #ifdef X_ENABLE_PIN
        #if X_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(X_ENABLE_PIN, enable.x);
        #else
            gpio_set_level(X_ENABLE_PIN, enable.x);
        #endif
      #endif
      #ifdef Y_ENABLE_PIN
        #if Y_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(Y_ENABLE_PIN, enable.y);
        #else
            gpio_set_level(Y_ENABLE_PIN, enable.y);
        #endif
      #endif
      #ifdef Z_ENABLE_PIN
        #if Z_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(Z_ENABLE_PIN, enable.z);
        #else
            gpio_set_level(Z_ENABLE_PIN, enable.z);
        #endif
      #endif
      #ifdef A_ENABLE_PIN
        #if A_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(A_ENABLE_PIN, enable.a);
        #else
            gpio_set_level(A_ENABLE_PIN, enable.a);
        #endif
      #endif
      #ifdef B_ENABLE_PIN
        #if B_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(B_ENABLE_PIN, enable.b);
        #else
            gpio_set_level(B_ENABLE_PIN, enable.b);
        #endif
      #endif
      #ifdef C_ENABLE_PIN
        #if C_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(C_ENABLE_PIN, enable.c);
        #else
            gpio_set_level(C_ENABLE_PIN, enable.c);
        #endif
      #endif
      #ifdef X2_ENABLE_PIN
        #if X2_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(X2_ENABLE_PIN, enable.x);
        #else
            gpio_set_level(X2_ENABLE_PIN, enable.x);
        #endif
      #endif
      #ifdef Y2_ENABLE_PIN
        #if Y2_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(Y2_ENABLE_PIN, enable.y);
        #else
            gpio_set_level(Y2_ENABLE_PIN, enable.y);
        #endif
      #endif
      #ifdef Z2_ENABLE_PIN
        #if Z2_ENABLE_PIN >= I2S_OUT_PIN_BASE
            DIGITAL_OUT(Z2_ENABLE_PIN, enable.z);
        #else
            gpio_set_level(Z2_ENABLE_PIN, enable.z);
        #endif
      #endif
    #endif
  #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0x00000000ULL);
//  timer_set_alarm_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 5000ULL);
#if GRBL_ESP32S3
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmhi.val = 0;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmlo.val = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up.
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_high = 0;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up.
#endif
    timer_start(STEP_TIMER_GROUP, STEP_TIMER_INDEX);
#if GRBL_ESP32S3
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.tn_alarm_en = TIMER_ALARM_EN;
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
#endif
}

// Sets up stepper driver interrupt timeout
IRAM_ATTR static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
// Limit min steps/s to about 2 (hal.f_step_timer @ 20MHz)
#if GRBL_ESP32S3
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmlo.val = cycles_per_tick < (1UL << 18) ? cycles_per_tick : (1UL << 18) - 1UL;
  #else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarmlo.val = cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL;
  #endif
#else
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 18) ? cycles_per_tick : (1UL << 18) - 1UL;
  #else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL;
  #endif
#endif
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline IRAM_ATTR static void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
#if X_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(X_DIRECTION_PIN, dir_outbits.x);
#else
    gpio_set_level(X_DIRECTION_PIN, dir_outbits.x);
#endif
#if Y_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(Y_DIRECTION_PIN, dir_outbits.y);
#else
    gpio_set_level(Y_DIRECTION_PIN, dir_outbits.y);
#endif
#ifdef Z_DIRECTION_PIN
  #if Z_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(Z_DIRECTION_PIN, dir_outbits.z);
  #else
    gpio_set_level(Z_DIRECTION_PIN, dir_outbits.z);
  #endif
#endif
#ifdef A_AXIS
  #if A_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(A_DIRECTION_PIN, dir_outbits.a);
  #else
    gpio_set_level(A_DIRECTION_PIN, dir_outbits.a);
  #endif
#endif
#ifdef B_AXIS
  #if B_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(B_DIRECTION_PIN, dir_outbits.b);
  #else
    gpio_set_level(B_DIRECTION_PIN, dir_outbits.b);
  #endif
#endif
#ifdef C_AXIS
  #if C_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(C_DIRECTION_PIN, dir_outbits.c);
  #else
    gpio_set_level(C_DIRECTION_PIN, dir_outbits.c);
  #endif
#endif
#ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
   #if X2_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(X2_DIRECTION_PIN, dir_outbits.x);
   #else
    gpio_set_level(X2_DIRECTION_PIN, dir_outbits.x);
   #endif
  #endif
  #ifdef Y2_DIRECTION_PIN
   #if Y2_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(Y2_DIRECTION_PIN, dir_outbits.y);
   #else
    gpio_set_level(Y2_DIRECTION_PIN, dir_outbits.y);
   #endif
  #endif
  #ifdef Z2_DIRECTION_PIN
   #if Z2_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(Z2_DIRECTION_PIN, dir_outbits.z);
   #else
    gpio_set_level(Z2_DIRECTION_PIN, dir_outbits.z);
   #endif
  #endif
#endif
}

#if USE_I2S_OUT

IRAM_ATTR static void I2S_stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    i2s_out_set_pulse_period((cycles_per_tick < (1UL << 18) ? cycles_per_tick : (1UL << 18) - 1UL) / (hal.f_step_timer / 1000000));
}

// Sets stepper direction and pulse pins and starts a step pulse
IRAM_ATTR static void I2S_stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change) {
        set_dir_outputs(stepper->dir_outbits);
        i2s_out_push_sample(i2s_delay_samples);
    }

    if(stepper->step_outbits.value) {
        i2s_set_step_outputs(stepper->step_outbits);
        i2s_out_push_sample(i2s_step_samples);
        i2s_set_step_outputs((axes_signals_t){0});
    }
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void I2S_stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});
    i2s_out_set_stepping();
}

#endif // USE_I2S_OUT

#ifdef SQUARING_ENABLED

#if USE_I2S_OUT

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

#else // RMT

// Set stepper pulse output pins
inline IRAM_ATTR static void set_step_outputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;
    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    if(step_outbits_1.x) {
        rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
        rmt_ll_tx_start(&RMT, X_AXIS);
    }
#ifdef X2_STEP_PIN
    if(step_outbits_2.x) {
        rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
        rmt_ll_tx_start(&RMT, X2_MOTOR);
    }
#endif

    if(step_outbits_1.y) {
        rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
        rmt_ll_tx_start(&RMT, Y_AXIS);
    }
#ifdef Y2_STEP_PIN
    if(step_outbits_2.y) {
        rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
        rmt_ll_tx_start(&RMT, Y2_MOTOR);
    }
#endif

#ifdef Z_STEP_PIN
    if(step_outbits_1.z) {
        rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
        rmt_ll_tx_start(&RMT, Z_AXIS);
    }
  #ifdef Z2_STEP_PIN
    if(step_outbits_2.z) {
        rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
        rmt_ll_tx_start(&RMT, Z2_MOTOR);
    }
  #endif
#endif // Z_STEP_PIN

#ifdef A_STEP_PIN
    if(step_outbits_1.a) {
        rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
        rmt_ll_tx_start(&RMT, A_AXIS);
    }
#endif

#ifdef B_STEP_PIN
    if(step_outbits_1.b) {
        rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
        rmt_ll_tx_start(&RMT, B_AXIS);
    }
#endif

#ifdef C_STEP_PIN
    if(step_outbits_1.c) {
        rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
        rmt_ll_tx_start(&RMT, C_AXIS);
    }
#endif
}

#endif // SQUARING_ENABLED

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else // SQUARING DISABLED

#if USE_I2S_OUT

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

#if STEP_INJECT_ENABLE

void stepperOutputStep (axes_signals_t step_outbits, axes_signals_t dir_outbits)
{
    if(step_outbits.value) {

        step_outbits.value ^= settings.steppers.step_invert.value;
        dir_outbits.value ^= settings.steppers.dir_invert.value;
  #ifdef GANGING_ENABLED
        axes_signals_t dir_outbits_2;
        dir_outbits_2.value = dir_outbits.value ^ settings.steppers.ganged_dir_invert.value;
  #endif

        if(step_outbits.x) {
            DIGITAL_OUT(X_DIRECTION_PIN, dir_outbits.x);
  #if X_GANGED
            DIGITAL_OUT(X2_DIRECTION_PIN, dir_outbits_2.x);
  #endif
        }

        if(step_outbits.y) {
            DIGITAL_OUT(Y_DIRECTION_PIN, dir_outbits.y);
  #if Y_GANGED
            DIGITAL_OUT(Y2_DIRECTION_PIN, dir_outbits_2.y);
  #endif
        }
  #ifdef Z_DIRECTION_PIN
        if(step_outbits.z) {
            DIGITAL_OUT(Z_DIRECTION_PIN, dir_outbits.z);
   #if Z_GANGED
            DIGITAL_OUT(Z2_DIRECTION_PIN, dir_outbits_2.z);
   #endif
        }
  #endif
#ifdef A_AXIS
        if(step_outbits.a)
            DIGITAL_OUT(A_DIRECTION_PIN, dir_outbits.a);
#endif
#ifdef B_AXIS
        if(step_outbits.b)
            DIGITAL_OUT(B_DIRECTION_PIN, dir_outbits.b);
#endif
#ifdef C_AXIS
        if(step_outbits.c)
            DIGITAL_OUT(C_DIRECTION_PIN, dir_outbits.c);
#endif

        if(step_outbits.x) {
            DIGITAL_OUT(X_STEP_PIN, step_outbits.x);
  #ifdef X2_STEP_PIN
            DIGITAL_OUT(X2_STEP_PIN, step_outbits.x);
  #endif
        }

        if(step_outbits.y) {
            DIGITAL_OUT(Y_STEP_PIN, step_outbits.y);
  #ifdef Y2_STEP_PIN
            DIGITAL_OUT(Y2_STEP_PIN, step_outbits.y);
  #endif
        }
  #ifdef Z_STEP_PIN
        if(step_outbits.z) {
            DIGITAL_OUT(Z_STEP_PIN, step_outbits.z);
   #ifdef Z2_STEP_PIN
            DIGITAL_OUT(Z2_STEP_PIN, step_outbits.z);
   #endif
        }
  #endif
#ifdef A_AXIS
        if(step_outbits.a)
            DIGITAL_OUT(A_STEP_PIN, step_outbits.a);
#endif
#ifdef B_AXIS
        if(step_outbits.b)
            DIGITAL_OUT(B_STEP_PIN, step_outbits.b);
#endif
#ifdef C_AXIS
        if(step_outbits.c)
            DIGITAL_OUT(C_STEP_PIN, step_outbits.c);
#endif
/*        while (esp_timer_get_time() - step_pulse_start_time < i2s_step_length) {
            __asm__ __volatile__ ("nop");  // spin here until time to turn off step
        } */
        i2s_out_push_sample(i2s_step_samples);
        i2s_set_step_outputs((axes_signals_t){0});
    }
}
#endif // STEP_INJECT_ENABLE

#else // RMT stepping

// Set stepper pulse output pins
inline IRAM_ATTR static void set_step_outputs (axes_signals_t step_outbits)
{
    if(step_outbits.x) {
        rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
        rmt_ll_tx_start(&RMT, X_AXIS);
#ifdef X2_STEP_PIN
        rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
        rmt_ll_tx_start(&RMT, X2_MOTOR);
#endif
    }

    if(step_outbits.y) {
        rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
        rmt_ll_tx_start(&RMT, Y_AXIS);
#ifdef Y2_STEP_PIN
        rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
        rmt_ll_tx_start(&RMT, Y2_MOTOR);
#endif
    }

#ifdef Z_STEP_PIN
    if(step_outbits.z) {
        rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
        rmt_ll_tx_start(&RMT, Z_AXIS);
  #ifdef Z2_STEP_PIN
        rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
        rmt_ll_tx_start(&RMT, Z2_MOTOR);
  #endif
    }
#endif

#ifdef A_STEP_PIN
    if(step_outbits.a) {
        rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
        rmt_ll_tx_start(&RMT, A_AXIS);
    }
#endif
#ifdef B_STEP_PIN
    if(step_outbits.b) {
        rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
        rmt_ll_tx_start(&RMT, B_AXIS);
    }
#endif
#ifdef C_STEP_PIN
    if(step_outbits.c) {
        rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
        rmt_ll_tx_start(&RMT, C_AXIS);
    }
#endif
}

#if STEP_INJECT_ENABLE

void stepperOutputStep (axes_signals_t step_outbits, axes_signals_t dir_outbits)
{
    if(step_outbits.value) {

        dir_outbits.value ^= settings.steppers.dir_invert.value;
  #ifdef GANGING_ENABLED
        axes_signals_t dir_outbits_2;
        dir_outbits_2.value = dir_outbits.value ^ settings.steppers.ganged_dir_invert.value;
  #endif

        if(step_outbits.x) {
            DIGITAL_OUT(X_DIRECTION_PIN, dir_outbits.x);
  #if X_GANGED
            DIGITAL_OUT(X2_DIRECTION_PIN, dir_outbits_2.x);
  #endif
        }

        if(step_outbits.y) {
            DIGITAL_OUT(Y_DIRECTION_PIN, dir_outbits.y);
  #if Y_GANGED
            DIGITAL_OUT(Y2_DIRECTION_PIN, dir_outbits_2.y);
  #endif
        }
  #ifdef Z_DIRECTION_PIN
        if(step_outbits.z) {
            DIGITAL_OUT(Z_DIRECTION_PIN, dir_outbits.z);
   #if Z_GANGED
            DIGITAL_OUT(Z2_DIRECTION_PIN, dir_outbits_2.z);
   #endif
        }
  #endif
#ifdef A_AXIS
        if(step_outbits.a)
            DIGITAL_OUT(A_DIRECTION_PIN, dir_outbits.a);
#endif
#ifdef B_AXIS
        if(step_outbits.b)
            DIGITAL_OUT(B_DIRECTION_PIN, dir_outbits.b);
#endif
#ifdef C_AXIS
        if(step_outbits.c)
            DIGITAL_OUT(C_DIRECTION_PIN, dir_outbits.c);
#endif

        if(step_outbits.x) {
            rmt_ll_tx_reset_pointer(&RMT, X_AXIS);
            rmt_ll_tx_start(&RMT, X_AXIS);
#ifdef X2_STEP_PIN
            rmt_ll_tx_reset_pointer(&RMT, X2_MOTOR);
            rmt_ll_tx_start(&RMT, X2_MOTOR);
#endif
        }

        if(step_outbits.y) {
            rmt_ll_tx_reset_pointer(&RMT, Y_AXIS);
            rmt_ll_tx_start(&RMT, Y_AXIS);
#ifdef Y2_STEP_PIN
            rmt_ll_tx_reset_pointer(&RMT, Y2_MOTOR);
            rmt_ll_tx_start(&RMT, Y2_MOTOR);
#endif
        }

#ifdef Z_STEP_PIN
        if(step_outbits.z) {
            rmt_ll_tx_reset_pointer(&RMT, Z_AXIS);
            rmt_ll_tx_start(&RMT, Z_AXIS);
  #ifdef Z2_STEP_PIN
            rmt_ll_tx_reset_pointer(&RMT, Z2_MOTOR);
            rmt_ll_tx_start(&RMT, Z2_MOTOR);
  #endif
        }
#endif

#ifdef A_AXIS
        if(step_outbits.a) {
            rmt_ll_tx_reset_pointer(&RMT, A_AXIS);
            rmt_ll_tx_start(&RMT, A_AXIS);
        }
#endif

#ifdef B_AXIS
        if(step_outbits.b) {
            rmt_ll_tx_reset_pointer(&RMT, B_AXIS);
            rmt_ll_tx_start(&RMT, B_AXIS);
        }
#endif

#ifdef C_AXIS
        if(step_outbits.c) {
            rmt_ll_tx_reset_pointer(&RMT, C_AXIS);
            rmt_ll_tx_start(&RMT, C_AXIS);
        }
#endif
    }
}

#endif // STEP_INJECT_ENABLE

#endif // RMT Stepping

#endif // SQUARING DISABLED

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
IRAM_ATTR static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change) {
        set_dir_outputs(stepper->dir_outbits);
#if USE_I2S_OUT
        uint64_t start_time = esp_timer_get_time();
        while (esp_timer_get_time() - start_time < i2s_delay_length) {
            __asm__ __volatile__ ("nop");  // spin here until time to output step
        }
#endif
    }

    if(stepper->step_outbits.value) {
#if USE_I2S_OUT
        uint64_t start_time = esp_timer_get_time();
        i2s_set_step_outputs(stepper->step_outbits);
        while (esp_timer_get_time() - start_time < i2s_step_length) {
            __asm__ __volatile__ ("nop");  // spin here until time to turn off step
        }
        i2s_set_step_outputs((axes_signals_t){0});
#else
        set_step_outputs(stepper->step_outbits);
#endif
    }
}

// Disables stepper driver interrupt
IRAM_ATTR static void stepperGoIdle (bool clear_signals)
{
#if GRBL_ESP32S3
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

void i2s_step_sink (void)
{
    //NOOP
}

void I2S_reset (void)
{
    if(goIdlePending) {
        i2s_out_set_passthrough();
        i2s_out_delay();
//      i2s_out_reset();
        goIdlePending = false;
    }
}

IRAM_ATTR static void I2S_stepperGoIdle (bool clear_signals)
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

static void i2s_set_streaming_mode (bool stream)
{
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.enable = 0;

    if(!stream && hal.stepper.wake_up == I2S_stepperWakeUp && i2s_out_get_pulser_status() == STEPPING) {
       i2s_out_set_passthrough();
       i2s_out_delay();
    }

    if(stream) {
        if(hal.stepper.wake_up != I2S_stepperWakeUp) {
            hal.stepper.wake_up = I2S_stepperWakeUp;
            hal.stepper.go_idle = I2S_stepperGoIdle;
            hal.stepper.cycles_per_tick = I2S_stepperCyclesPerTick;
            hal.stepper.pulse_start = I2S_stepperPulseStart;
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
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
    axes_signals_t pin;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        i--;
        if(inputpin[i].group & (PinGroup_Limit|PinGroup_LimitMax)) {
            if(on && homing_cycle.mask) {
                pin = xbar_fn_to_axismask(inputpin[i].id);
                disable = inputpin[i].group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
            }
            gpio_set_intr_type(inputpin[i].pin, on ? map_intr_type(inputpin[i].irq_mode) : GPIO_INTR_DISABLE);
            if(disable)
                gpio_intr_disable(inputpin[i].pin);
            else
                gpio_intr_enable(inputpin[i].pin);
        }
    } while(i);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef X_LIMIT_PIN
    signals.min.x = gpio_get_level(X_LIMIT_PIN);
#endif
#ifdef Y_LIMIT_PIN
    signals.min.y = gpio_get_level(Y_LIMIT_PIN);
#endif
#ifdef Z_LIMIT_PIN
    signals.min.z = gpio_get_level(Z_LIMIT_PIN);
#endif
#ifdef A_LIMIT_PIN
    signals.min.a = gpio_get_level(A_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = gpio_get_level(B_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = gpio_get_level(C_LIMIT_PIN);
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = gpio_get_level(X2_LIMIT_PIN);
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = gpio_get_level(Y2_LIMIT_PIN);
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = gpio_get_level(Z2_LIMIT_PIN);
#endif

    if (settings.limits.invert.value) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

#ifdef RESET_PIN
    signals.reset = gpio_get_level(RESET_PIN);
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = gpio_get_level(FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = gpio_get_level(CYCLE_START_PIN);
#endif
#ifdef SAFETY_DOOR_PIN
    signals.safety_door_ajar = gpio_get_level(SAFETY_DOOR_PIN);
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

#ifdef PROBE_PIN

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away, bool probing)
{
#if USE_I2S_OUT
    i2s_set_streaming_mode(!(probing || laser_mode));
#endif

    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

#if PROBE_ISR
    gpio_set_intr_type(inputpin[INPUT_PROBE].pin, probe_invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE);
    inputpin[INPUT_PROBE].active = false;
#endif
}

// Returns the probe connected and triggered pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;

#if PROBE_ISR
    // TODO: verify!
    inputpin[INPUT_PROBE].active = inputpin[INPUT_PROBE].active || ((uint8_t)gpio_get_level(PROBE_PIN) ^ probe.inverted);
    state.triggered = inputpin[INPUT_PROBE].active;
#else
    state.triggered = (uint8_t)gpio_get_level(PROBE_PIN) ^ probe.inverted;
#endif

    return state;
}

#endif

#if DRIVER_SPINDLE_ENABLE

// Static spindle (off, on cw & on ccw)
IRAM_ATTR inline static void spindle_off (void)
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? On : Off;
    ioexpand_out(iopins);
#elif defined(SPINDLE_ENABLE_PIN)
  #if SPINDLE_ENABLE_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0);
  #else
    gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0);
  #endif
#endif
}

IRAM_ATTR static void spindleOffBasic (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    spindle_off();
}

IRAM_ATTR inline static void spindle_on (void)
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? Off : On;
    ioexpand_out(iopins);
#elif defined(SPINDLE_ENABLE_PIN)
  #if SPINDLE_ENABLE_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : 1);
  #else
    gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : 1);
  #endif
#endif
}

IRAM_ATTR inline static void spindle_dir (bool ccw)
{
#if IOEXPAND_ENABLE
    iopins.spindle_dir = (ccw ^ settings.spindle.invert.ccw) ? On : Off;
    ioexpand_out(iopins);
#elif defined(SPINDLE_DIRECTION_PIN)
  #if SPINDLE_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(SPINDLE_DIRECTION_PIN, (ccw ^ settings.spindle.invert.ccw) ? 1 : 0);
  #else
    gpio_set_level(SPINDLE_DIRECTION_PIN, (ccw ^ settings.spindle.invert.ccw) ? 1 : 0);
  #endif
#endif
}

// Start or stop spindle
IRAM_ATTR static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

#if DRIVER_SPINDLE_PWM_ENABLE

// Variable spindle control functions

// Sets spindle speed
IRAM_ATTR static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == pwm(spindle)->off_value) {
        if(pwm(spindle)->settings->flags.enable_rpm_controlled) {
            if(pwm(spindle)->cloned)
                spindle_dir(false);
            else
                spindle_off();
        }
#if PWM_RAMPED
        pwm_ramp.pwm_target = pwm_value;
        ledc_set_fade_step_and_start(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
        if(spindle_pwm.always_on) {
            ledc_set_duty(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel, pwm(spindle)->off_value);
            ledc_update_duty(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel);
        } else
            ledc_stop(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel, pwm(spindle)->settings->invert.pwm ? 1 : 0);
#endif
        pwmEnabled = false;
     } else {
#if PWM_RAMPED
         pwm_ramp.pwm_target = pwm_value;
         ledc_set_fade_step_and_start(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
         ledc_set_duty(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel, pwm(spindle)->settings->invert.pwm ? pwm_max_value - pwm_value : pwm_value);
         ledc_update_duty(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel);
#endif
        if(!pwmEnabled) {
            if(pwm(spindle)->cloned)
                spindle_dir(true);
            else
                spindle_on();
            pwmEnabled = true;
        }
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return pwm(spindle)->compute_value(pwm(spindle), rpm, false);
}

// Start or stop spindle, variable version

IRAM_ATTR static void spindleOff (spindle_ptrs_t *spindle)
{
    spindle_off();
    if(spindle)
        spindleSetSpeed(spindle, pwm(spindle)->off_value);
}

IRAM_ATTR static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
#ifdef SPINDLE_DIRECTION_PIN
    if(state.on || pwm(spindle)->cloned)
        spindle_dir(state.ccw);
#endif
    if(!pwm(spindle)->settings->flags.enable_rpm_controlled) {
        if(state.on)
            spindle_on();
        else
            spindle_off();
    }

    spindleSetSpeed(spindle, state.on || (state.ccw && pwm(spindle)->cloned)
                              ? pwm(spindle)->compute_value(pwm(spindle), rpm, false)
                              : pwm(spindle)->off_value);
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if((spindle->cap.variable = !settings.spindle.flags.pwm_disable && settings.spindle.rpm_max > settings.spindle.rpm_min)) {

        spindle->esp32_off = spindleOff;
        spindle->set_state = spindleSetStateVariable;

        if(spindle_pwm_timer.freq_hz != (uint32_t)settings.spindle.pwm_freq) {
            spindle_pwm_timer.freq_hz = (uint32_t)settings.spindle.pwm_freq;
            if(spindle_pwm_timer.freq_hz <= 100) {
#if SOC_LEDC_TIMER_BIT_WIDE_NUM > 14
                if(spindle_pwm_timer.duty_resolution != LEDC_TIMER_16_BIT) {
                    spindle_pwm_timer.duty_resolution = LEDC_TIMER_16_BIT;
                    ledc_timer_config(&spindle_pwm_timer);
                }
#else
                if(spindle_pwm_timer.duty_resolution != LEDC_TIMER_14_BIT) {
                    spindle_pwm_timer.duty_resolution = LEDC_TIMER_14_BIT;
                    ledc_timer_config(&spindle_pwm_timer);
                }
#endif
            } else if(spindle_pwm_timer.duty_resolution != LEDC_TIMER_10_BIT) {
                spindle_pwm_timer.duty_resolution = LEDC_TIMER_10_BIT;
                ledc_timer_config(&spindle_pwm_timer);
            }
        }

        pwm_max_value = (1UL << spindle_pwm_timer.duty_resolution) - 1;
        spindle_pwm.offset = (settings.spindle.invert.pwm ? -1 : 1);
        spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.spindle, pwm_max_value * settings.spindle.pwm_freq);

        ledc_set_freq(spindle_pwm_timer.speed_mode, spindle_pwm_timer.timer_num, spindle_pwm_timer.freq_hz);

    } else {
        if(pwmEnabled)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->esp32_off = spindleOffBasic;
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

#if USE_I2S_OUT
    if(spindle->id == spindle_get_default())
        i2s_set_streaming_mode(!(laser_mode = spindle->cap.laser));
#endif

    return true;
}

#endif // DRIVER_SPINDLE_PWM_ENABLE

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    UNUSED(spindle);

#if IOEXPAND_ENABLE // TODO: read from expander?
    state.on = iopins.spindle_on;
    state.ccw = iopins.spindle_dir;
#else
 #if defined(SPINDLE_ENABLE_PIN)
  #if SPINDLE_ENABLE_PIN >= I2S_OUT_PIN_BASE
    state.on = DIGITAL_IN(SPINDLE_ENABLE_PIN) != 0;
  #else
    state.on = gpio_get_level(SPINDLE_ENABLE_PIN) != 0;
  #endif
 #endif
 #if defined(SPINDLE_DIRECTION_PIN)
  #if SPINDLE_DIRECTION_PIN >= I2S_OUT_PIN_BASE
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PIN) != 0;
  #else
    state.ccw = gpio_get_level(SPINDLE_DIRECTION_PIN) != 0;
  #endif
 #endif
#endif
    state.value ^= settings.spindle.invert.mask;
#if DRIVER_SPINDLE_PWM_ENABLE
    state.on |= pwmEnabled;
  #if PWM_RAMPED
    state.at_speed = ledc_get_duty(spindle_pwm_channel.speed_mode, spindle_pwm_channel.channel) == pwm_ramp.pwm_target;
  #endif
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

// Start/stop coolant (and mist if enabled)
IRAM_ATTR static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
#if IOEXPAND_ENABLE
    iopins.flood_on = mode.flood;
    iopins.mist_on = mode.mist;
    ioexpand_out(iopins);
#else
 #ifdef COOLANT_FLOOD_PIN
  #if COOLANT_FLOOD_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(COOLANT_FLOOD_PIN, mode.flood ? 1 : 0);
  #else
    gpio_set_level(COOLANT_FLOOD_PIN, mode.flood ? 1 : 0);
  #endif
 #endif
 #ifdef COOLANT_MIST_PIN
  #if COOLANT_MIST_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_OUT(COOLANT_MIST_PIN, mode.mist ? 1 : 0);
  #else
    gpio_set_level(COOLANT_MIST_PIN, mode.mist ? 1 : 0);
  #endif
 #endif
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {settings.coolant_invert.mask};

#if IOEXPAND_ENABLE // TODO: read from expander?
    state.flood = iopins.flood_on;
    state.mist = iopins.mist_on;
#else
 #ifdef COOLANT_FLOOD_PIN
  #if COOLANT_FLOOD_PIN >= I2S_OUT_PIN_BASE
    DIGITAL_IN(COOLANT_FLOOD_PIN);
  #else
    state.flood = gpio_get_level(COOLANT_FLOOD_PIN);
  #endif
 #endif
 #ifdef COOLANT_MIST_PIN
  #if COOLANT_MIST_PIN >= I2S_OUT_PIN_BASE
    state.mist = DIGITAL_IN(COOLANT_MIST_PIN);
  #else
    state.mist = gpio_get_level(COOLANT_MIST_PIN);
  #endif
 #endif
#endif

    state.value ^= settings.coolant_invert.mask;

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

#if MPG_MODE == 1

static void modeChange(sys_state_t state)
{
    stream_mpg_enable(!gpio_get_level(MPG_ENABLE_PIN));
}

static void modeEnable (sys_state_t state)
{
    if(sys.mpg_mode == gpio_get_level(MPG_ENABLE_PIN))
        stream_mpg_enable(true);
}

#endif

void debounceTimerCallback (TimerHandle_t xTimer)
{
    uint32_t grp = 0, i = sizeof(inputpin) / sizeof(input_signal_t);
    do {
        i--;
        if(inputpin[i].debounce && inputpin[i].active) {
            inputpin[i].active = false; //gpio_get_level(inputpin[i].pin) == (inputpin[i].invert ? 0 : 1);
            grp |= inputpin[i].group;
        }
    } while(i);

    if(grp & (PinGroup_Limit|PinGroup_LimitMax)) {
        portENTER_CRITICAL(&debounce_mux);
        hal.limits.interrupt_callback(limitsGetState());
        portEXIT_CRITICAL(&debounce_mux);
    }

    if(grp & PinGroup_Control) {
        portENTER_CRITICAL(&debounce_mux);
        hal.control.interrupt_callback(systemGetState());
        portEXIT_CRITICAL(&debounce_mux);
    }
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

#if DRIVER_SPINDLE_PWM_ENABLE
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

#if USE_I2S_OUT
        i2s_delay_length = (uint32_t)settings->steppers.pulse_delay_microseconds;
        i2s_step_length = (uint32_t)settings->steppers.pulse_microseconds;

        if(i2s_delay_length < I2S_OUT_USEC_PER_PULSE)
            i2s_delay_length = I2S_OUT_USEC_PER_PULSE;
        else if(i2s_delay_length > 10)
            i2s_delay_length = 10;

        if(i2s_step_length < I2S_OUT_USEC_PER_PULSE)
            i2s_step_length = I2S_OUT_USEC_PER_PULSE;
        else if(i2s_step_length > 20)
            i2s_step_length = 20;

        i2s_delay_samples = i2s_delay_length / I2S_OUT_USEC_PER_PULSE; // round up?
        i2s_step_samples = i2s_step_length / I2S_OUT_USEC_PER_PULSE; // round up?
#else
        initRMT(settings);
#endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup = true;
        control_signals_t control_fei;
        gpio_config_t config;
        input_signal_t *signal;

        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

        do {

            signal = &inputpin[--i];
            signal->irq_mode = IRQ_Mode_None;

            switch(signal->id) {

                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    signal->invert = control_fei.reset;
                    break;

                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    signal->invert = control_fei.feed_hold;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    signal->invert = control_fei.cycle_start;
                    break;

                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    signal->invert = control_fei.safety_door_ajar;
                    break;

                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    signal->invert = false;
                    break;

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    pullup = !settings->limits.disable_pullup.x;
                    signal->invert = limit_fei.x;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    pullup = !settings->limits.disable_pullup.y;
                    signal->invert = limit_fei.y;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    pullup = !settings->limits.disable_pullup.z;
                    signal->invert = limit_fei.z;
                   break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                    pullup = !settings->limits.disable_pullup.a;
                    signal->invert = limit_fei.a;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
                    pullup = !settings->limits.disable_pullup.b;
                    signal->invert = limit_fei.b;
                    break;
#endif
#ifdef C_LIMIT_PIN
                case Input_LimitC:
                    pullup = !settings->limits.disable_pullup.c;
                    signal->invert = limit_fei.c;
                    break;
#endif
#if MPG_MODE == 1
                case Input_ModeSelect:
                    pullup = true;
                    signal->invert = false;
                    config.intr_type = GPIO_INTR_ANYEDGE;
  #if ETHERNET_ENABLE
                    gpio_isr_handler_add(signal->pin, gpio_mpg_isr, signal);
  #endif
                    break;
#endif
#if I2C_STROBE_ENABLE
                case Input_I2CStrobe:
                    pullup = true;
                    signal->invert = false;
                    config.intr_type = GPIO_INTR_ANYEDGE;
  #if ETHERNET_ENABLE
                    gpio_isr_handler_add(signal->pin, gpio_i2c_strobe_isr, signal);
  #endif
                    break;
#endif
                default:
                    break;
            }

            switch(signal->group) {

                case PinGroup_Control:
                case PinGroup_Limit:
                case PinGroup_LimitMax:
                    signal->irq_mode = signal->invert ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    signal->debounce = hal.driver_cap.software_debounce;
#if ETHERNET_ENABLE
                    gpio_isr_handler_add(signal->pin, (signal->group & (PinGroup_Limit|PinGroup_LimitMax)) ? gpio_limit_isr : gpio_control_isr, signal);
#endif
                    break;

                case PinGroup_AuxInput:
                    pullup = true;
                    signal->invert = false;
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
                config.pull_up_en = pullup && signal->pin < 34 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                config.pull_down_en = pullup || signal->pin >= 34 ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
                config.intr_type = (signal->group & (PinGroup_Limit|PinGroup_LimitMax)) ? GPIO_INTR_DISABLE : map_intr_type(signal->irq_mode);

                signal->offset = config.pin_bit_mask > (1ULL << 31) ? 1 : 0;
                signal->mask = signal->offset == 0 ? (uint32_t)config.pin_bit_mask : (uint32_t)(config.pin_bit_mask >> 32);

    //            printf("IN %d - %d - %d : %x\n", signal->pin,  signal->offset, signal->mask, signal->invert);

                gpio_config(&config);

                signal->active = signal->debounce && gpio_get_level(signal->pin) == (signal->invert ? 0 : 1);
            }
        } while(i);
    }
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {0};
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.pin = outputpin[i].pin - (outputpin[i].pin < I2S_OUT_PIN_BASE ? 0 : I2S_OUT_PIN_BASE);
        pin.port = low_level || outputpin[i].pin < I2S_OUT_PIN_BASE ? NULL : "I2S";
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);

        ppin = ppin->next;
    } while(ppin);
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

static bool bus_ok = false;
static sdmmc_card_t *card = NULL;

static bool sdcard_unmount (FATFS **fs)
{
    if(card && esp_vfs_fat_sdcard_unmount("/sdcard", card) == ESP_OK) {
        card = NULL;
        bus_ok = false;
        spi_bus_free(SDSPI_DEFAULT_HOST);
    }

    return card == NULL;
}

static char *sdcard_mount (FATFS **fs)
{
    if(!bus_ok) {

        spi_bus_config_t bus_config = {
            .mosi_io_num     = PIN_NUM_MOSI,
            .miso_io_num     = PIN_NUM_MISO,
            .sclk_io_num     = PIN_NUM_CLK,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = 4000,
            .flags           = SPICOMMON_BUSFLAG_MASTER,
            .intr_flags      = ESP_INTR_FLAG_IRAM
        };

#if PIN_NUM_CLK == GPIO_NUM_14
        if(spi_bus_initialize(SPI2_HOST, &bus_config, 1) != ESP_OK)
            return NULL;
#elif PIN_NUM_CLK == GPIO_NUM_18
        if(spi_bus_initialize(SPI3_HOST, &bus_config, 1) != ESP_OK)
            return NULL;
#else
        if(spi_bus_initialize(SDSPI_DEFAULT_HOST, &bus_config, 1) != ESP_OK)
            return NULL;
#endif

        bus_ok = true;
    }

    if(!bus_ok)
        return NULL;

    if(card == NULL) {

        esp_err_t ret = ESP_FAIL;
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        };

        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
//        host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = PIN_NUM_CS;
        slot_config.host_id = host.slot;

        gpio_set_drive_capability(PIN_NUM_CS, GPIO_DRIVE_CAP_3);

        if ((ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card)) != ESP_OK)
            report_message(ret == ESP_FAIL ? "Failed to mount filesystem" : "Failed to initialize SD card", Message_Warning);
    }

    if(card && fs) {
        if(*fs == NULL)
            *fs = malloc(sizeof(FATFS));

        if(*fs && f_mount(*fs, "", 1) != FR_OK) {
           free(*fs );
           *fs  = NULL;
        }
    }

    return "";
}

#endif

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{

#if LITTLEFS_ENABLE
    fs_littlefs_mount("/littlefs", esp32_littlefs_hal());
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
    timer_isr_register(STEP_TIMER_GROUP, STEP_TIMER_INDEX, stepper_driver_isr, 0, ESP_INTR_FLAG_IRAM, NULL);
    timer_enable_intr(STEP_TIMER_GROUP, STEP_TIMER_INDEX);

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
        if(outputpin[idx].id == Output_SdCardCS)
            continue;
#if USE_I2S_OUT
        else if(outputpin[idx].pin >= I2S_OUT_PIN_BASE)
            outputpin[idx].mode = Pin_I2S;
#endif
        else if((outputpin[idx].mode = outputpin[idx].group == PinGroup_StepperStep ? Pin_RMT : Pin_GPIO) == Pin_GPIO)
            mask |= (1ULL << outputpin[idx].pin);

    } while(idx);

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
            if(outputpin[idx].mode == Pin_I2S)
                DIGITAL_OUT(outputpin[idx].pin, 1);
            else
#endif
            gpio_set_level(outputpin[idx].pin, 1);
        }
    } while(idx);

#if MPG_MODE == 1

    /************************
     *  MPG mode (pre)init  *
     ************************/

    // Set as output low (until boot is complete)
    gpioConfig.pin_bit_mask = (1ULL << MPG_ENABLE_PIN);
    gpio_config(&gpioConfig);
    gpio_set_level(MPG_ENABLE_PIN, 0);

#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce)
        debounceTimer = xTimerCreate("debounce", pdMS_TO_TICKS(32), pdFALSE, NULL, debounceTimerCallback);

    /******************************************
     *  Control, limit & probe pins dir init  *
     ******************************************/

#if ETHERNET_ENABLE
    gpio_install_isr_service(0);
#else
    gpio_isr_register(gpio_isr, NULL, (int)ESP_INTR_FLAG_IRAM, NULL);
#endif

#if DRIVER_SPINDLE_PWM_ENABLE

    /******************
    *  Spindle init  *
    ******************/

#if PWM_RAMPED
    ledc_fade_func_install(ESP_INTR_FLAG_IRAM);
#endif
    spindle_pwm_channel.speed_mode = spindle_pwm_timer.speed_mode;
    ledc_timer_config(&spindle_pwm_timer);
    ledc_channel_config(&spindle_pwm_channel);

    static const periph_pin_t pwm = {
        .function = Output_SpindlePWM,
        .group = PinGroup_SpindlePWM,
        .pin = SPINDLE_PWM_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    hal.periph_port.register_pin(&pwm);

    /**/

#endif // DRIVER_SPINDLE_PWM_ENABLE

#if SDCARD_ENABLE

    sdcard_events_t *card = sdcard_init();
    card->on_mount = sdcard_mount;
    card->on_unmount = sdcard_unmount;

    sdcard_mount(NULL);

    static const periph_pin_t sck = {
        .function = Output_SCK,
        .group = PinGroup_SPI,
        .pin = PIN_NUM_CLK,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t sdo = {
        .function = Output_MOSI,
        .group = PinGroup_SPI,
        .pin = PIN_NUM_MOSI,
        .mode = { .mask = PINMODE_NONE }
    };

    static const periph_pin_t sdi = {
        .function = Input_MISO,
        .group = PinGroup_SPI,
        .pin = PIN_NUM_MISO,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&sck);
    hal.periph_port.register_pin(&sdo);
    hal.periph_port.register_pin(&sdi);

#endif

#if IOEXPAND_ENABLE
    ioexpand_init();
#endif

  // Set defaults

    IOInitDone = settings->version == 22;

    hal.settings_changed(settings, (settings_changed_flags_t){0});
    hal.stepper.go_idle(true);

#if USE_I2S_OUT && defined(SPINDLE_PWM_PIN)
    on_spindle_selected = grbl.on_spindle_selected;
    grbl.on_spindle_selected = onSpindleSelected;
#endif

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

    return (rtc_started = settimeofday(&t, &tz) == 0);
}

static bool get_rtc_time (struct tm *dt)
{
    bool ok = false;

    if(rtc_started) {
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

    if(xTaskGetTickCountFromISR() - ms > 250) {
        ms = xTaskGetTickCountFromISR();
        vTaskDelay(1);
    }
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    rtc_cpu_freq_config_t cpu;
    rtc_clk_cpu_freq_get_config(&cpu);
#if GRBL_ESP32S3
    hal.info = "ESP32-S3";
#else
    hal.info = "ESP32";
#endif
    hal.driver_version = "231218";
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
    hal.settings_changed = settings_changed;

#if !USE_I2S_OUT
    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
  #if STEP_INJECT_ENABLE
    hal.stepper.output_step = stepperOutputStep;
  #endif
#else
    hal.driver_reset = I2S_reset;
    hal.stepper.wake_up = I2S_stepperWakeUp;
    hal.stepper.go_idle = I2S_stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = I2S_stepperCyclesPerTick;
    hal.stepper.pulse_start = I2S_stepperPulseStart;
    i2s_out_init();
    i2s_out_set_pulse_callback(hal.stepper.interrupt_callback);
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

#ifdef PROBE_PIN
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
#endif

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

    hal.rtc.get_datetime = get_rtc_time;
    hal.rtc.set_datetime = set_rtc_time;

    serialRegisterStreams();

    grbl.on_execute_realtime = wdt_tickler;

#if USB_SERIAL_CDC
    stream_connect(usb_serialInit());
#else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
#endif

#if I2C_ENABLE
    I2CInit();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else
    if(nvsInit()) {
        hal.nvs.type = NVS_Flash;
        hal.nvs.memcpy_from_flash = nvsRead;
        hal.nvs.memcpy_to_flash = nvsWrite;
    } else
        hal.nvs.type = NVS_None;
#endif

#ifdef DEBUGOUT

    hal.debug_out = debug_out;
#endif

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_PWM_ENABLE

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
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
  #if IOEXPAND_ENABLE || DRIVER_SPINDLE_DIR_ENABLE
            .direction = On,
  #endif
  #if PWM_RAMPED
            .at_speed = On
  #endif

        }
    };

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .esp32_off = spindleOffBasic,
        .cap = {
            .gpio_controlled = On,
  #if IOEXPAND_ENABLE || DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

  // driver capabilities, used for announcing and negotiating (with the core) driver functionality

  #if IOEXPAND_ENABLE || defined(COOLANT_MIST_PIN)
    hal.driver_cap.mist_control = On;
  #endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();

    uint32_t i;
    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};
    input_signal_t *input;
    output_signal_t *output;

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.irq_mode = IRQ_Mode_Edges;
            aux_inputs.n_pins++;
        }
    }

    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            aux_outputs.n_pins++;
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if MPG_MODE == 1
  #if KEYPAD_ENABLE == 2
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), true, keypad_enqueue_keycode)))
        protocol_enqueue_rt_command(modeEnable);
  #else
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), true, NULL)))
        protocol_enqueue_rt_command(modeEnable);
  #endif
#elif MPG_MODE == 2
    hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), true, keypad_enqueue_keycode);
#elif KEYPAD_ENABLE == 2
    stream_open_instance(KEYPAD_STREAM, 115200, keypad_enqueue_keycode);
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

#include "grbl/plugins_init.h"

    // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
IRAM_ATTR static void stepper_driver_isr (void *arg)
{
#if GRBL_ESP32S3
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
    if(((input_signal_t *)signal)->debounce) {
        ((input_signal_t *)signal)->active = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(debounceTimer, &xHigherPriorityTaskWoken);
    } else
        hal.limits.interrupt_callback(limitsGetState());
}

IRAM_ATTR static void gpio_control_isr (void *signal)
{
    if(((input_signal_t *)signal)->debounce) {
        ((input_signal_t *)signal)->active = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(debounceTimer, &xHigherPriorityTaskWoken);
    } else
        hal.control.interrupt_callback(systemGetState());
}

IRAM_ATTR static void gpio_aux_isr (void *signal)
{
    ioports_event((input_signal_t *)signal);
}

#if MPG_MODE == 1

IRAM_ATTR static void gpio_mpg_isr (void *signal)
{
    static bool mpg_mutex = false;

    if(!mpg_mutex) {
        mpg_mutex = true;
        protocol_enqueue_rt_command(modeChange);
        mpg_mutex = false;
    }
}

#endif

#if I2C_STROBE_ENABLE
IRAM_ATTR static void gpio_i2c_strobe_isr (void *signal)
{
    if(i2c_strobe.callback)
        i2c_strobe.callback(0, gpio_get_level(I2C_STROBE_PIN));
}
#endif

#else // ETHERNET_ENABLE

  //GPIO intr process
IRAM_ATTR static void gpio_isr (void *arg)
{
    bool debounce = false;
    uint32_t grp = 0, intr_status[2];
    intr_status[0] = READ_PERI_REG(GPIO_STATUS_REG);          // get interrupt status for GPIO0-31
    intr_status[1] = READ_PERI_REG(GPIO_STATUS1_REG);         // get interrupt status for GPIO32-39
    SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG, intr_status[0]);  // clear intr for gpio0-gpio31
    SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG, intr_status[1]); // clear intr for gpio32-39

    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
    do {
        i--;
        if(intr_status[inputpin[i].offset] & inputpin[i].mask) {

            if(inputpin[i].group & PinGroup_AuxInput)
                ioports_event(&inputpin[i]);
            else {
                inputpin[i].active = true;
                if(inputpin[i].debounce)
                    debounce = true;
                else
                    grp |= inputpin[i].group;
            }
        }
    } while(i);

    if(debounce) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(debounceTimer, &xHigherPriorityTaskWoken);
    }

    if(grp & (PinGroup_Limit|PinGroup_LimitMax))
        hal.limits.interrupt_callback(limitsGetState());

    if(grp & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());

#if MPG_MODE == 1

    static bool mpg_mutex = false;

    if((grp & PinGroup_MPG) && !mpg_mutex) {
        mpg_mutex = true;
        protocol_enqueue_rt_command(modeChange);
        mpg_mutex = false;
    }
#endif

#if I2C_STROBE_ENABLE
    if((grp & PinGroup_Keypad) && i2c_strobe.callback)
        i2c_strobe.callback(0, gpio_get_level(I2C_STROBE_PIN));
#endif
}

#endif
