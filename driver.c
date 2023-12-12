/*

  driver.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2022 Terje Io
  Copyright (c) 2021 Volksolive

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

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/sio.h"

#include "driver.h"
#include "serial.h"
#include "driverPIO.pio.h"

#include "grbl/crossbar.h"
#include "grbl/limits.h"
#include "grbl/state_machine.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/protocol.h"

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if KEYPAD_ENABLE == 2
#include "keypad/keypad.h"
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
#endif

#if PPI_ENABLE
#include "laser/ppi.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

#if defined(STEP_PIN_BASE_X) || defined(STEP_PIN_BASE_Y) || defined(STEP_PIN_BASE_Z)
#define STEP_SINGLE_PIO
  #ifdef STEP_PIN_BASE_X
    static uint pulse_x;
  #endif
  #ifdef STEP_PIN_BASE_Y
    static uint pulse_y;
  #endif
  #ifdef STEP_PIN_BASE_Z
    static uint pulse_z;
  #endif
#endif

typedef union {
    uint32_t value;
    struct {
        uint32_t delay  :8,
                 length :8,
                 set    :6,
                 reset  :6;
    };
} pio_steps_t;

static pio_steps_t pio_steps = { .delay = 20, .length = 100 };
static uint pulse, timer;
static uint16_t pulse_length, pulse_delay;
static bool pwmEnabled = false, IOInitDone = false;
static const io_stream_t *serial_stream;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static status_code_t (*on_unknown_sys_command)(uint_fast16_t state, char *line, char *lcline);
static volatile uint32_t elapsed_ticks = 0;
static volatile bool ms_event = false;
static probe_state_t probe = {
    .connected = On
};
static pin_group_pins_t limit_inputs;
#ifdef SAFETY_DOOR_PIN
static input_signal_t *safety_door;
#endif

#if IOEXPAND_ENABLE
static ioexpand_t io_expander = {0};
#endif

#include "grbl/stepdir_map.h"

static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,          .port = GPIO_INPUT, .pin = RESET_PIN,           .group = PinGroup_Control },
#else
    { .id = Input_Reset,          .port = GPIO_INPUT, .pin = RESET_PIN,           .group = PinGroup_Control },
#endif
#ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,       .port = GPIO_INPUT, .pin = FEED_HOLD_PIN,       .group = PinGroup_Control },
#endif
#ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,     .port = GPIO_INPUT, .pin = CYCLE_START_PIN,     .group = PinGroup_Control },
#endif
#ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,     .port = GPIO_INPUT, .pin = SAFETY_DOOR_PIN,     .group = PinGroup_Control },
#endif
#ifdef LIMITS_OVERRIDE_PIN
    { .id = Input_LimitsOverride, .port = GPIO_INPUT, .pin = LIMITS_OVERRIDE_PIN, .group = PinGroup_Control },
#endif
    { .id = Input_Probe,          .port = GPIO_INPUT, .pin = PROBE_PIN,           .group = PinGroup_Probe },
    { .id = Input_LimitX,         .port = GPIO_INPUT, .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,       .port = GPIO_INPUT, .pin = X2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitY,         .port = GPIO_INPUT, .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_Max,     .port = GPIO_INPUT, .pin = Y2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitZ,         .port = GPIO_INPUT, .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit }
#ifdef Z2_LIMIT_PIN
  , { .id = Input_LimitZ_Max,     .port = GPIO_INPUT, .pin = Z2_LIMIT_PIN,        .group = PinGroup_Limit }
#endif
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,         .port = GPIO_INPUT, .pin = A_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,         .port = GPIO_INPUT, .pin = B_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef C_LIMIT_PIN
  , { .id = Input_LimitC,         .port = GPIO_INPUT, .pin = C_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#if MPG_MODE_PIN
  ,  { .id = Input_MPGSelect,     .port = GPIO_INPUT, .pin = MPG_MODE_PIN,        .group = PinGroup_MPG }
#endif
#if I2C_STROBE_ENABLE && defined(I2C_STROBE_PIN)
  , { .id = Input_KeypadStrobe,   .port = GPIO_INPUT, .pin = I2C_STROBE_PIN,      .group = PinGroup_Keypad }
#endif
#ifdef AUX_INPUT0_PIN
  , { .id = Input_Aux0,           .port = GPIO_INPUT, .pin = AUX_INPUT0_PIN,      .group = PinGroup_AuxInput }
#endif
#ifdef AUX_INPUT1_PIN
  , { .id = Input_Aux1,           .port = GPIO_INPUT, .pin = AUX_INPUT1_PIN,      .group = PinGroup_AuxInput }
#endif
#ifdef AUX_INPUT2_PIN
  , { .id = Input_Aux2,           .port = GPIO_INPUT, .pin = AUX_INPUT2_PIN,      .group = PinGroup_AuxInput }
#endif
#ifdef AUX_INPUT3_PIN
  , { .id = Input_Aux3,           .port = GPIO_INPUT, .pin = AUX_INPUT3_PIN,      .group = PinGroup_AuxInput }
#endif
#ifdef AUX_INPUT4_PIN
  , { .id = Input_Aux4,           .port = GPIO_INPUT, .pin = AUX_INPUT4_PIN,      .group = PinGroup_AuxInput }
#endif
};

static output_signal_t outputpin[] = {
#ifdef STEP_PINS_BASE
    { .id = Output_StepX,           .port = GPIO_PIO,                   .pin = STEP_PINS_BASE,          .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepY,           .port = GPIO_PIO,                   .pin = STEP_PINS_BASE + 1,      .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepZ,           .port = GPIO_PIO,                   .pin = STEP_PINS_BASE + 2,      .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = GPIO_PIO,                   .pin = STEP_PINS_BASE + 3,      .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = GPIO_PIO,                   .pin = STEP_PINS_BASE + 4,      .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = GPIO_PIO,                   .pin = STEP_PINS_BASE + 5,      .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = GPIO_PIO,                   .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = GPIO_PIO,                   .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = GPIO_PIO,                   .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#endif
#ifdef DIRECTION_PORT
    { .id = Output_DirX,            .port = DIRECTION_PORT,             .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirY,            .port = DIRECTION_PORT,             .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirZ,            .port = DIRECTION_PORT,             .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = DIRECTION_PORT,             .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = DIRECTION_PORT,             .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = DIRECTION_PORT,             .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = DIRECTION_PORT,             .pin = X2_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = DIRECTION_PORT,             .pin = Y2_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = DIRECTION_PORT,             .pin = Z2_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#endif
#if !TRINAMIC_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,      .pin = STEPPERS_ENABLE_PIN,    .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef X_STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X_STEPPERS_ENABLE_PORT,    .pin = X_STEPPERS_ENABLE_PIN,  .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Y_STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y_STEPPERS_ENABLE_PORT,    .pin = Y_STEPPERS_ENABLE_PIN,  .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Z_STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_STEPPERS_ENABLE_PORT,    .pin = Z_STEPPERS_ENABLE_PIN,  .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef A_STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_STEPPERS_ENABLE_PORT,    .pin = A_STEPPERS_ENABLE_PIN,  .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef B_STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_STEPPERS_ENABLE_PORT,    .pin = B_STEPPERS_ENABLE_PIN,  .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef C_STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_STEPPERS_ENABLE_PORT,    .pin = C_STEPPERS_ENABLE_PIN,  .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#endif
#if !VFD_SPINDLE
#ifdef SPINDLE_PWM_PIN
    { .id = Output_SpindlePWM,      .port = SPINDLE_PWM_PORT,           .pin = SPINDLE_PWM_PIN,         .group = PinGroup_SpindlePWM },
#endif
#ifdef SPINDLE_ENABLE_PIN
    { .id = Output_SpindleOn,       .port = SPINDLE_ENABLE_PORT,        .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
#endif
#ifdef SPINDLE_DIRECTION_PIN
    { .id = Output_SpindleDir,      .port = SPINDLE_DIRECTION_PORT,     .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
#endif
#endif
#ifdef COOLANT_FLOOD_PIN
    { .id = Output_CoolantFlood,    .port = COOLANT_FLOOD_PORT,         .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
#endif
#ifdef COOLANT_MIST_PIN
    { .id = Output_CoolantMist,     .port = COOLANT_MIST_PORT,          .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#endif
#ifdef SD_CS_PIN
    { .id = Output_SdCardCS,        .port = GPIO_OUTPUT,                .pin = SD_CS_PIN,               .group = PinGroup_SdCard },
#endif
#ifdef SD_CS_PORT
    { .id = Output_SdCardCS,        .port = SD_CS_PORT,                 .pin = SD_CS_PIN,               .group = PinGroup_SdCard },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,            .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,            .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,            .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#if SD_SHIFT_REGISTER
    { .id = Output_StepX,           .port = GPIO_SR8,  .pin = 0,  .group = PinGroup_StepperStep },
    { .id = Output_StepY,           .port = GPIO_SR8,  .pin = 1,  .group = PinGroup_StepperStep },
    { .id = Output_StepZ,           .port = GPIO_SR8,  .pin = 2,  .group = PinGroup_StepperStep },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = GPIO_SR8,  .pin = 3,  .group = PinGroup_StepperStep },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = GPIO_SR8,  .pin = 3,  .group = PinGroup_StepperStep },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = GPIO_SR8,  .pin = 3,  .group = PinGroup_StepperStep },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = GPIO_SR8,  .pin = 3,  .group = PinGroup_StepperStep },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = GPIO_SR8,  .pin = 3,  .group = PinGroup_StepperStep },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = GPIO_SR8,  .pin = 3,  .group = PinGroup_StepperStep },
#endif
#endif
    { .id = Output_DirX,            .port = GPIO_SR8,  .pin = 4,  .group = PinGroup_StepperDir },
    { .id = Output_DirY,            .port = GPIO_SR8,  .pin = 5,  .group = PinGroup_StepperDir },
    { .id = Output_DirZ,            .port = GPIO_SR8,  .pin = 6,  .group = PinGroup_StepperDir },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = GPIO_SR8,  .pin = 7,  .group = PinGroup_StepperDir },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = GPIO_SR8,  .pin = 7,  .group = PinGroup_StepperDir },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = GPIO_SR8,  .pin = 7,  .group = PinGroup_StepperDir },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = GPIO_SR8,  .pin = 7,  .group = PinGroup_StepperDir },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = GPIO_SR8,  .pin = 7,  .group = PinGroup_StepperDir },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = GPIO_SR8,  .pin = 7,  .group = PinGroup_StepperDir },
#endif
#if OUT_SHIFT_REGISTER
    { .id = Output_StepperEnableX,  .port = GPIO_SR16, .pin = 0,  .group = PinGroup_StepperEnable },
    { .id = Output_StepperEnableY,  .port = GPIO_SR16, .pin = 1,  .group = PinGroup_StepperEnable },
    { .id = Output_StepperEnableZ,  .port = GPIO_SR16, .pin = 2,  .group = PinGroup_StepperEnable },
#ifdef A_AXIS
    { .id = Output_StepperEnableA,  .port = GPIO_SR16, .pin = 3,  .group = PinGroup_StepperEnable },
#endif
#ifdef B_AXIS
    { .id = Output_StepperEnableB,  .port = GPIO_SR16, .pin = 3,  .group = PinGroup_StepperEnable },
#endif
#ifdef C_AXIS
    { .id = Output_StepperEnableC,  .port = GPIO_SR16, .pin = 3,  .group = PinGroup_StepperEnable },
#endif
    { .id = Output_SpindleOn,       .port = GPIO_SR16, .pin = 4,  .group = PinGroup_SpindleControl },
    { .id = Output_SpindleDir,      .port = GPIO_SR16, .pin = 5,  .group = PinGroup_SpindleControl },
    { .id = Output_CoolantFlood,    .port = GPIO_SR16, .pin = 6,  .group = PinGroup_Coolant },
    { .id = Output_CoolantMist,     .port = GPIO_SR16, .pin = 7,  .group = PinGroup_Coolant },
    { .id = Output_Aux0,            .port = GPIO_SR16, .pin = 8,  .group = PinGroup_AuxOutput },
    { .id = Output_Aux1,            .port = GPIO_SR16, .pin = 9,  .group = PinGroup_AuxOutput },
    { .id = Output_Aux2,            .port = GPIO_SR16, .pin = 10, .group = PinGroup_AuxOutput },
    { .id = Output_Aux3,            .port = GPIO_SR16, .pin = 11, .group = PinGroup_AuxOutput },
    { .id = Output_Aux4,            .port = GPIO_SR16, .pin = 12, .group = PinGroup_AuxOutput },
    { .id = Output_Aux5,            .port = GPIO_SR16, .pin = 13, .group = PinGroup_AuxOutput },
    { .id = Output_Aux6,            .port = GPIO_SR16, .pin = 14, .group = PinGroup_AuxOutput },
    { .id = Output_Aux7,            .port = GPIO_SR16, .pin = 15, .group = PinGroup_AuxOutput },
#endif
};

#ifndef STEPPERS_ENABLE_MASK
#define STEPPERS_ENABLE_MASK 0
#endif

#if KEYPAD_ENABLE == 0
#define KEYPAD_STROBE_BIT 0
#endif

#if !SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_BIT 0
#endif

// This should be a sdk function but it doesn't exist yet
#define gpio_set_irqover(gpio, value) hw_write_masked(&iobank0_hw->io[gpio].ctrl, value << IO_BANK0_GPIO0_CTRL_IRQOVER_LSB, IO_BANK0_GPIO0_CTRL_IRQOVER_BITS);

#define NVIC_HIGH_LEVEL_PRIORITY 0xC0
#define NVIC_MEDIUM_LEVEL_PRIORITY 0x80
#define NVIC_LOW_LEVEL_PRIORITY 0x40

#define DRIVER_IRQMASK (LIMIT_MASK|CONTROL_MASK|KEYPAD_STROBE_BIT|SPINDLE_INDEX_BIT)

#define LIMIT_DEBOUNCE_TEMPO    40  // 40ms for Limit debounce
#define SR_LATCH_DEBOUNCE_TEMPO 40  // 40ms for SR LATCH 

/*
#define DEBOUNCE_ALARM_HW_TIMER 0   // Hardware alarm timer 0 used for the debounce alarm pool
#define DEBOUNCE_ALARM_MAX_TIMER 16 // Maximum number of alarm timer in the debounce alarm pool (based on SDK 'PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS 16' for default pool used for driver_delay in driver.c)

typedef struct {
    alarm_id_t id;
    uint8_t pin;
    uint8_t level;
} debounce_pool_t;
static alarm_pool_t *debounceAlarmPool;
static volatile debounce_pool_t debounceAlarmPoolArray[DEBOUNCE_ALARM_MAX_TIMER];

*/

#if SD_SHIFT_REGISTER
static step_dir_sr_t sd_sr;
#endif

#if OUT_SHIFT_REGISTER
static output_sr_t out_sr;
#endif

static void systick_handler (void);
static void stepper_int_handler (void);
static void gpio_int_handler (uint gpio, uint32_t events);
static void spindle_set_speed (uint_fast16_t pwm_value);

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

static int64_t delay_callback(alarm_id_t id, void *callback)
{
    ((delay_callback_ptr)callback)();

    return 0;
}

static void driver_delay (uint32_t ms, delay_callback_ptr callback)
{
    if(ms > 0) {
        if(callback)
            add_alarm_in_ms(ms, delay_callback, callback, false);
        else {
            uint32_t delay = ms * 1000, start = timer_hw->timerawl;
            while(timer_hw->timerawl - start < delay)
                grbl.on_execute_delay(state_get());
        }
    } else if(callback)
        callback();
        tight_loop_contents();
}

//*************************  STEPPER  *************************//

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
#elif STEPPERS_ENABLE_OUTMODE
  #if STEPPERS_ENABLE_OUTMODE == GPIO_IOEXPAND
    #ifdef STEPPERS_DISABLEX_PIN
    ioex_out(STEPPERS_DISABLEX_PIN) = enable.x;
    #endif
    #ifdef STEPPERS_DISABLEZ_PIN
    ioex_out(STEPPERS_DISABLEZ_PIN) = enable.z;
    #endif
    ioexpand_out(io_expander);
  #elif OUT_SHIFT_REGISTER
    out_sr.x_ena = enable.x;
   #ifdef X2_ENABLE_PIN
    out_sr.m3_ena = enable.x;
   #endif
    out_sr.y_ena = enable.y;
   #ifdef Y2_ENABLE_PIN
    out_sr.m3_ena = enable.y;
   #endif
    out_sr.z_ena = enable.z;
   #ifdef Z2_ENABLE_PIN
    out_sr.m3_ena = enable.z;
   #endif
   #ifdef A_ENABLE_PIN
    out_sr.m3_ena = enable.a;
   #endif
    out_sr16_write(pio1, 1, out_sr.value);
  #else
    gpio_put(STEPPERS_ENABLE_PIN, enable.x);
  #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});
    stepper_timer_set_period(pio1, 0, timer, 1000);
    irq_set_enabled(PIO1_IRQ_0, true);
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    irq_set_enabled(PIO1_IRQ_0, false);
    stepper_timer_stop(pio1, 0);
}

// Sets up stepper driver interrupt timeout, "Normal" version
static void __not_in_flash_func(stepperCyclesPerTick)(uint32_t cycles_per_tick)
{
    stepper_timer_set_period(pio1, 0, timer, cycles_per_tick < 1000000 ? cycles_per_tick : 1000000);
}

#ifdef SQUARING_ENABLED

static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;

#if SD_SHIFT_REGISTER
    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    sd_sr.set.x_step = step_outbits_1.x;
  #ifdef X2_STEP_PIN
    sd_sr.set.m3_step = step_outbits_2.x;
  #endif
    sd_sr.set.y_step = step_outbits_1.y;
  #ifdef Y2_STEP_PIN
    sd_sr.set.m3_step = step_outbits_2.y;
  #endif
    sd_sr.set.z_step = step_outbits_1.z;
  #ifdef Z2_STEP_PIN
    sd_sr.set.m3_step = step_outbits_2.z;
  #endif
  #ifdef A_STEP_PIN
    sd_sr.set.m3_step = step_outbits_1.a;
  #endif
    step_dir_sr4_write(pio0, 0, sd_sr.value);
#else
    pio_steps.set = step_outbits_1.mask ^ settings.steppers.step_invert.mask;
  #ifdef X2_STEP_PIN
    if(step_outbits_1.x ^ settings.steppers.step_invert.x)
        pio_steps.set |= X2_STEP_BIT;
  #endif
  #ifdef Y2_STEP_PIN
    if(step_outbits_1.y ^ settings.steppers.step_invert.y)
        pio_steps.set |= Y2_STEP_BIT;
  #endif
  #ifdef Z2_STEP_PIN
    if(step_outbits_1.z ^ settings.steppers.step_invert.z)
        pio_steps.set |= Z2_STEP_BIT;
  #endif
    step_pulse_generate(pio0, 0, pio_steps.value);
#endif
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#ifdef STEP_SINGLE_PIO
    step_outbits.mask ^= settings.steppers.step_invert.mask;
  #ifdef STEP_PIN_BASE_X
    pio_steps.set = step_outbits.x;
    pio_steps.reset = settings.steppers.step_invert.mask.x;
    step_pulse_generate(pio0, pulse_x, pio_steps.value);
  #endif
  #ifdef STEP_PIN_BASE_Y
    pio_steps.set = step_outbits.y;
    pio_steps.reset = settings.steppers.step_invert.mask.y;
    step_pulse_generate(pio0, pulse_y, pio_steps.value);
  #endif
  #ifdef STEP_PIN_BASE_Z
   pio_steps.set = step_outbits.z;
   pio_steps.reset = settings.steppers.step_invert.mask.z;
   step_pulse_generate(pio0, pulse_z, pio_steps.value);
  #endif
#elif SD_SHIFT_REGISTER
    step_outbits.mask ^= settings.steppers.step_invert.mask;
    sd_sr.set.x_step = step_outbits.x;
  #ifdef X2_STEP_PIN
    sd_sr.set.m3_step = step_outbits.x;
  #endif
    sd_sr.set.y_step = step_outbits.y;
  #ifdef Y2_STEP_PIN
    sd_sr.set.m3_step = step_outbits.y;
  #endif
    sd_sr.set.z_step = step_outbits.z;
  #ifdef Z2_STEP_PINs
    sd_sr.set.m3_step = step_outbits.z;
  #endif
  #ifdef A_STEP_PIN
    sd_sr.set.m3_step = step_outbits.a;
  #endif
    step_dir_sr4_write(pio0, 0, sd_sr.value);
#else
    pio_steps.set = step_outbits.mask ^ settings.steppers.step_invert.mask;
    //pio_steps.set = 0b1101 ^ settings.steppers.step_invert.mask;
  #ifdef X2_STEP_PIN
    if(step_outbits.x ^ settings.steppers.step_invert.x)
        pio_steps.set |= X2_STEP_BIT;
  #endif
  #ifdef Y2_STEP_PIN
    if(step_outbits.y ^ settings.steppers.step_invert.y)
        pio_steps.set |= Y2_STEP_BIT;
  #endif
  #ifdef Z2_STEP_PIN
    if(step_outbits.z ^ settings.steppers.step_invert.z)
        pio_steps.set |= Z2_STEP_BIT;
  #endif
    step_pulse_generate(pio0, 0, pio_steps.value);
#endif
}

#endif // SQUARING_ENABLED

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

#endif

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
//inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_outbits)
static  void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if SD_SHIFT_REGISTER
    dir_outbits.mask ^= settings.steppers.dir_invert.mask;
    sd_sr.set.x_dir = sd_sr.reset.x_dir = dir_outbits.x;
    sd_sr.set.y_dir = sd_sr.reset.y_dir = dir_outbits.y;
    sd_sr.set.z_dir = sd_sr.reset.z_dir = dir_outbits.z;
 #ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = dir_outbits.x;
  #endif
  #ifdef Y2_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = dir_outbits.y;
  #endif
  #ifdef Z2_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = dir_outbits.z;
  #endif
 #endif
  #ifdef A_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = dir_outbits.a;
  #endif
    // dir signals are set on the next step pulse output
#elif DIRECTION_OUTMODE == GPIO_MAP
    gpio_put_masked(DIRECTION_MASK, dir_outmap[dir_outbits.mask]);
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_BIT, (dir_outbits.x ^ settings.steppers.dir_invert.x) ^ settings.steppers.ganged_dir_invert.mask.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_BIT, (dir_outbits.y ^ settings.steppers.dir_invert.y) ^ settings.steppers.ganged_dir_invert.mask.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_BIT, (dir_outbits.z ^ settings.steppers.dir_invert.z) ^ settings.steppers.ganged_dir_invert.mask.z);
  #endif
#else
    dir_outbits.mask ^= settings.steppers.dir_invert.mask;
    gpio_put_masked(DIRECTION_MASK, dir_outbits.mask << DIRECTION_OUTMODE);
 #ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_BIT, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_BIT, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_BIT, dir_outbits.z);
  #endif
 #endif
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void __not_in_flash_func(stepperPulseStart)(stepper_t *stepper)
{
    if(stepper->dir_change)
        stepperSetDirOutputs(stepper->dir_outbits);

    if(stepper->step_outbits.value)
        stepperSetStepOutputs(stepper->step_outbits);
}

//*************************  LIMIT  *************************//

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    uint32_t i = limit_inputs.n_pins;

    on = on && settings.limits.flags.hard_enabled;

    do {
        i--;
        pinEnableIRQ(&limit_inputs.pins.inputs[i], on ? limit_inputs.pins.inputs[i].irq_mode : IRQ_Mode_None);
    } while(i);

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an limit_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};

#if LIMIT_INMODE == GPIO_MAP
    signals.min.x = DIGITAL_IN(X_LIMIT_BIT);
  #ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_BIT);
  #endif
    signals.min.y = DIGITAL_IN(Y_LIMIT_BIT);
  #ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_BIT);
  #endif
    signals.min.z = DIGITAL_IN(Z_LIMIT_BIT);
  #ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_BIT);
  #endif
  #ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_BIT);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_BIT);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(C_LIMIT_BIT);
  #endif
#endif

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t __not_in_flash_func(systemGetState) (void)
{
    control_signals_t signals = {0};

#if CONTROL_INMODE == GPIO_MAP
  #ifdef ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_BIT);
  #else                                   
    signals.reset = DIGITAL_IN(RESET_BIT);
  #endif
  #ifdef FEED_HOLD_PIN
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_BIT);
  #endif
  #ifdef CYCLE_START_PIN
    signals.cycle_start = DIGITAL_IN(CYCLE_START_BIT);
  #endif
  #ifdef SAFETY_DOOR_PIN
    signals.safety_door_ajar = safety_door->active;
  #endif
#endif

    return signals;
}

//*************************  PROBE  *************************//

#ifdef PROBE_PIN

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.triggered = false;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

    gpio_set_inover(PROBE_PIN, probe.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

    if((probe.is_probing = probing))
        gpio_set_irq_enabled(PROBE_PIN, probe.inverted ? GPIO_IRQ_LEVEL_LOW : GPIO_IRQ_LEVEL_HIGH, true);
    else
        gpio_set_irq_enabled(PROBE_PIN, GPIO_IRQ_ALL, false);
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = probe.is_probing ? probe.triggered : DIGITAL_IN(PROBE_BIT);

    return state;
}

#endif

//*************************  SPINDLE  *************************//

// Static spindle (off, on cw & on ccw)
inline static void spindle_off (void)
{
#if OUT_SHIFT_REGISTER
    out_sr.spindle_ena = settings.spindle.invert.on;
    out_sr16_write(pio1, 1, out_sr.value);
#elif SPINDLE_OUTMODE == GPIO_IOEXPAND
    ioex_out(SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
    ioexpand_out(io_expander);
#else
    #ifdef SPINDLE_ENABLE_PIN
    DIGITAL_OUT(SPINDLE_ENABLE_BIT, Off);
    #endif
#endif
}

inline static void spindle_on (void)
{
#if OUT_SHIFT_REGISTER
    out_sr.spindle_ena = !settings.spindle.invert.on;
    out_sr16_write(pio1, 1, out_sr.value);
#elif SPINDLE_OUTMODE == GPIO_IOEXPAND
    ioex_out(SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
    ioexpand_out(io_expander);
#else
    #ifdef SPINDLE_ENABLE_PIN
    DIGITAL_OUT(SPINDLE_ENABLE_BIT, On);
    #endif
#endif
#if SPINDLE_SYNC_ENABLE
    spindleDataReset();
#endif
}

inline static void spindle_dir (bool ccw)
{
#if OUT_SHIFT_REGISTER
    out_sr.spindle_dir = ccw ^ settings.spindle.invert.ccw;
    out_sr16_write(pio1, 1, out_sr.value);
#elif SPINDLE_OUTMODE == GPIO_IOEXPAND
    if(hal.driver_cap.spindle_dir) {
        ioex_out(SPINDLE_DIRECTION_PIN) = ccw ^ settings.spindle.invert.ccw;
        ioexpand_out(io_expander);
    }
#else
    #ifdef SPINDLE_DIRECTION_PIN
    if(hal.driver_cap.spindle_dir)
        DIGITAL_OUT(SPINDLE_DIRECTION_BIT, ccw);
    #endif
#endif
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

// Variable spindle control functions

// Sets spindle speed
static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
        if(spindle_pwm.always_on) 
            pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle_pwm.off_value);
        else
            pwm_set_gpio_level(SPINDLE_PWM_PIN, 0);
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        pwm_set_gpio_level(SPINDLE_PWM_PIN, pwm_value);
    }
}

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else

static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

#endif

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    if (!state.on || rpm == 0.0f) {
        spindle_set_speed(spindle_pwm.off_value);
        spindle_off();
    } else {
        spindle_dir(state.ccw);
        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
    }
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {settings.spindle.invert.mask};

#if OUT_SHIFT_REGISTER
    state.on = out_sr.spindle_ena;
    state.ccw = out_sr.spindle_dir;
    state.value ^= settings.spindle.invert.mask;

#elif SPINDLE_OUTMODE == GPIO_IOEXPAND
    state.on = ioex_out(SPINDLE_ENABLE_PIN);
    state.ccw = hal.driver_cap.spindle_dir && ioex_out(SPINDLE_DIRECTION_PIN);
    state.value ^= settings.spindle.invert.mask;
#else
    #ifdef SPINDLE_DIRECTION_PIN
    state.on = DIGITAL_IN(SPINDLE_ENABLE_BIT);
    #endif
    #ifdef SPINDLE_DIRECTION_PIN
    state.ccw = hal.driver_cap.spindle_dir && DIGITAL_IN(SPINDLE_DIRECTION_BIT);
    #endif
#endif

    return state;
}

void driver_spindle_pwm_init (void) {

    if(hal.driver_cap.variable_spindle) {

        hal.spindle.set_state = spindleSetStateVariable;

        // Get the default config for 
        pwm_config config = pwm_get_default_config();
        
        uint32_t prescaler = settings.spindle.pwm_freq > 2000.0f ? 1 : (settings.spindle.pwm_freq > 200.0f ? 12 : 50);

        spindle_precompute_pwm_values(&spindle_pwm, clock_get_hz(clk_sys) / prescaler);

        // Set divider, not using the 4 fractional bit part of the clock divider, only the integer part
        pwm_config_set_clkdiv_int(&config, prescaler);
        // Set the top value of the PWM => the period
        pwm_config_set_wrap(&config, spindle_pwm.period);
        // Set the off value of the PWM => off duty cycle (either 0 or the off value)
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle_pwm.off_value);

        // Set polarity of the channel
        uint channel = pwm_gpio_to_channel(SPINDLE_PWM_PIN);                                                                          // Get which is associated with the PWM pin
        pwm_config_set_output_polarity(&config, (!channel & settings.spindle.invert.pwm), (channel & settings.spindle.invert.pwm));   // Set the polarity of the pin's channel

        // Load the configuration into our PWM slice, and set it running.
        pwm_init(pwm_gpio_to_slice_num(SPINDLE_PWM_PIN), &config, true);
    } else
        hal.spindle.set_state = spindleSetState;
}

#if PPI_ENABLE

static void spindlePulseOn (uint_fast16_t pulse_length)
{
//    PPI_TIMER->ARR = pulse_length;
//    PPI_TIMER->EGR = TIM_EGR_UG;
//    PPI_TIMER->CR1 |= TIM_CR1_CEN;
    spindle_on();
}

#endif

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
#if OUT_SHIFT_REGISTER
    mode.value ^= settings.coolant_invert.mask;
    out_sr.flood_ena = mode.flood;
    out_sr.mist_ena = mode.mist;
    out_sr16_write(pio1, 1, out_sr.value);
#elif COOLANT_OUTMODE == GPIO_IOEXPAND
    mode.value ^= settings.coolant_invert.mask;
    ioex_out(COOLANT_FLOOD_PIN) = mode.flood;
    #ifdef COOLANT_MIST_PIN
    ioex_out(COOLANT_MIST_PIN) = mode.mist;
    #endif
    ioexpand_out(io_expander);
#else
    #ifdef COOLANT_FLOOD_PIN
    DIGITAL_OUT(COOLANT_FLOOD_BIT, mode.flood);
    #endif
    #ifdef COOLANT_MIST_PIN
    DIGITAL_OUT(COOLANT_MIST_BIT, mode.mist);
    #endif
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};
#if OUT_SHIFT_REGISTER
    state.flood = out_sr.flood_ena;
    state.mist = out_sr.mist_ena;
    state.value ^= settings.coolant_invert.mask;
#elif COOLANT_OUTMODE == GPIO_IOEXPAND
    state.value = settings.coolant_invert.mask;
    state.flood = ioex_out(COOLANT_FLOOD_PIN);
    #ifdef COOLANT_MIST_PIN
    state.mist = ioex_out(COOLANT_MIST_PIN);
    state.value ^= settings.coolant_invert.mask;
    #endif
#else
    #ifdef COOLANT_FLOOD_PIN
    state.flood = DIGITAL_IN(COOLANT_FLOOD_BIT);
    #endif
    #ifdef COOLANT_MIST_PIN
    state.mist  = DIGITAL_IN(COOLANT_MIST_BIT);
    #endif
#endif

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
   __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();
    return prev;
}

static uint32_t getElapsedTicks (void)
{
   return elapsed_ticks;
}

#if MPG_MODE == 1

static input_signal_t *mpg_pin = NULL;

static void mpg_select (sys_state_t state)
{
    stream_mpg_enable(DIGITAL_IN(mpg_pin->bit) == 0);

    pinEnableIRQ(mpg_pin, (mpg_pin->irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

static void mpg_enable (sys_state_t state)
{
    if(sys.mpg_mode != (DIGITAL_IN(mpg_pin->bit) == 0))
        mpg_select(state);
    else
        pinEnableIRQ(mpg_pin, (mpg_pin->irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

#endif
/*
// Save the gpio that has generated the IRQ together with the alarm pool id and the edge
static void debounce_alarm_pool_save_gpio (alarm_id_t id, uint pin, uint level)
{
    static volatile uint8_t index = 0;

    debounceAlarmPoolArray[index].id = id;
    debounceAlarmPoolArray[index].pin = pin;
    debounceAlarmPoolArray[index].level = level;
    index = (index + 1) % DEBOUNCE_ALARM_MAX_TIMER;
}
*/

void pinEnableIRQ (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    switch(irq_mode) {

        case IRQ_Mode_Rising:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_EDGE_RISE, true);
            break;
        
        case IRQ_Mode_Falling:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_EDGE_FALL, true);
            break;

        case IRQ_Mode_Change:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
            break;

        case IRQ_Mode_Low:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_LEVEL_LOW, true);
            break;

        case IRQ_Mode_High:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_LEVEL_HIGH, true);
            break;

        case IRQ_Mode_None:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_ALL, false);
            break;
    }
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{

#ifdef SPINDLE_PWM_PIN
    hal.driver_cap.variable_spindle = settings->spindle.rpm_min < settings->spindle.rpm_max;
#endif

#if USE_STEPDIR_MAP
    stepdirmap_init(settings);
#endif

    stepperSetStepOutputs((axes_signals_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    if(IOInitDone) {

        // Init of the spindle PWM
        driver_spindle_pwm_init();


#if SD_SHIFT_REGISTER
        pio_steps.length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - 0.8f));
        pio_steps.delay = settings->steppers.pulse_delay_microseconds <= 0.8f
                           ? 2 : (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - 0.8f));
        sr_delay_set(pio0, 1, pio_steps.delay);
        sr_hold_set(pio0, 2, pio_steps.length);
        sd_sr.reset.x_step = settings->steppers.step_invert.x;
  #ifdef X2_DIRECTION_PIN
        sd_sr.reset.m3_step = settings->steppers.step_invert.x;
  #endif
        sd_sr.reset.y_step = settings->steppers.step_invert.y;
  #ifdef Y2_DIRECTION_PIN
        sd_sr.reset.m3_step = settings->steppers.step_invert.y;
  #endif
        sd_sr.reset.z_step = settings->steppers.step_invert.z;
  #ifdef Z2_DIRECTION_PIN
        sd_sr.reset.m3_step = settings->steppers.step_invert.z;
  #endif
  #ifdef A_DIRECTION_PIN
        sd_sr.reset.m3_step = settings->steppers.step_invert.a;
  #endif

#else // PIO step parameters init

        pio_steps.length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds)) - 1;
        pio_steps.delay = settings->steppers.pulse_delay_microseconds == 0.0f
                           ? 1 : (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds)) - 1;
        pio_steps.reset = settings->steppers.step_invert.mask;
  #ifdef X2_STEP_PIN
        if(settings->steppers.step_invert.x)
            pio_steps.reset |= X2_STEP_BIT;
  #endif
  #ifdef Y2_STEP_PIN
        if(settings->steppers.step_invert.y)
            pio_steps.reset |= Y2_STEP_BIT;
  #endif
  #ifdef Z2_STEP_PIN
        if(settings->steppers.step_invert.z)
            pio_steps.reset |= Z2_STEP_BIT;
  #endif

#endif

        /***********************
         *  Input pins config  *
         ***********************/

        // Set the GPIO interrupt handler, the pin doesn't matter for now
        gpio_set_irq_enabled_with_callback(0, 0, false, gpio_int_handler); 

        // Disable GPIO IRQ while initializing the input pins
        irq_set_enabled(IO_IRQ_BANK0, false);
        
        bool pullup;
        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *input;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {

            input = &inputpin[--i];
            input->bit = 1 << input->pin;
            input->debounce = false;
            input->invert = false;
 
            input->irq_mode = IRQ_Mode_None;
            pullup = input->group == PinGroup_AuxInput;

            gpio_init(input->pin);
            if(input->group != PinGroup_Limit)
                gpio_set_irq_enabled(input->pin, GPIO_IRQ_ALL, false);

            switch(input->id) {

                case Input_EStop:
                    pullup = !settings->control_disable_pullup.e_stop;
                    input->invert = control_fei.e_stop;
                    break;

                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    input->invert = control_fei.reset;
                    break;

                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    input->invert = control_fei.feed_hold;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    input->invert = control_fei.cycle_start;
                    break;
#ifdef SAFETY_DOOR_PIN
                case Input_SafetyDoor:
                    safety_door = input;
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    input->invert = control_fei.safety_door_ajar;
                    input->active = DIGITAL_IN(input->bit);
                    input->irq_mode = safety_door->invert ? IRQ_Mode_Low : IRQ_Mode_High);
                    break;
#endif
                case Input_Probe:
                    pullup = !settings->probe.disable_probe_pullup;
                    input->invert = settings->probe.invert_probe_pin;
                    break;

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    pullup = !settings->limits.disable_pullup.x;
                    input->invert = limit_fei.x;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    pullup = !settings->limits.disable_pullup.y;
                    input->invert = limit_fei.y;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    pullup = !settings->limits.disable_pullup.z;
                    input->invert = limit_fei.z;
                    break;

                case Input_LimitA:
                case Input_LimitA_Max:
                    pullup = !settings->limits.disable_pullup.a;
                    input->invert = limit_fei.a;
                    break;

                case Input_LimitB:
                case Input_LimitB_Max:
                    pullup = !settings->limits.disable_pullup.b;
                    input->invert = limit_fei.b;
                    break;

                case Input_LimitC:
                case Input_LimitC_Max:
                    pullup = !settings->limits.disable_pullup.c;
                    input->invert = limit_fei.c;
                    break;
#ifdef MPG_MODE_PIN
                case Input_MPGSelect:
                    pullup = true;
                    mpg_pin = input;
                    break;
#endif
                case Input_KeypadStrobe:
                    pullup = true;
                    input->irq_mode = IRQ_Mode_Change;
                    break;

                default:
                    break;
            }

            switch(input->group) {
 
                case PinGroup_Limit:
                case PinGroup_Control:
                    input->irq_mode = input->invert ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case PinGroup_AuxInput:
                    pullup = true;
                    input->cap.irq_mode = IRQ_Mode_All;
                    break;

                default:
                    break;
            }

            gpio_set_pulls(input->pin, pullup, !pullup);
            gpio_set_inover(input->pin, input->invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

            if(input->group != PinGroup_Limit)
                pinEnableIRQ(input, input->irq_mode);

            if(input->id == Input_Probe)
                probeConfigure(false, false);

            gpio_acknowledge_irq(input->pin, GPIO_IRQ_ALL);
        } while(i);

        /*************************
         *  Output signals init  *
         *************************/
        
        for(uint_fast8_t i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
            if(outputpin[i].port == GPIO_OUTPUT) switch(outputpin[i].id) {

                case Output_SpindleOn:
                    gpio_set_outover(outputpin[i].pin, settings->spindle.invert.on ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;
 
                case Output_SpindleDir:
                    gpio_set_outover(outputpin[i].pin, settings->spindle.invert.ccw ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;

                case Output_CoolantMist:
                    gpio_set_outover(outputpin[i].pin, settings->coolant_invert.mist ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;

                case Output_CoolantFlood:
                    gpio_set_outover(outputpin[i].pin, settings->coolant_invert.flood ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;
            }
        }

        //Activate GPIO IRQ
        irq_set_priority(IO_IRQ_BANK0, NVIC_MEDIUM_LEVEL_PRIORITY); // By default all IRQ are medium priority but in case the GPIO IRQ would need high or low priority it can be done here 
        irq_set_enabled(IO_IRQ_BANK0, true);                        // Enable GPIO IRQ
    }
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info)
{
    static xbar_t pin = {0};
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
//        pin.port = low_level ? NULL : (void *)port2char(inputpin[i].port);
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.port = inputpin[i].port == GPIO_SR8 ? (void *)"SR8" : (inputpin[i].port == GPIO_SR16 ? (void *)"SR16" : NULL);
        pin.description = inputpin[i].description;

        pin_info(&pin);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = outputpin[i].port == GPIO_PIO ? (void *)"PIO" :
                   (outputpin[i].port == GPIO_IOEXPAND ? (void *)"IOX" :
                    (outputpin[i].port == GPIO_SR8 ? (void *)"SR8." :
                     (outputpin[i].port == GPIO_SR16 ? (void *)"SR16." : NULL)));
        pin.description = outputpin[i].description;

        pin_info(&pin);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin);

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

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    /*************************
     *  Output signals init  *
     *************************/

    for(uint_fast8_t i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(outputpin[i].port == GPIO_OUTPUT) {
            outputpin[i].bit = 1 << outputpin[i].pin;
            gpio_init(outputpin[i].pin);
            gpio_set_dir_out_masked(outputpin[i].bit);
            if(outputpin[i].group == PinGroup_SpindlePWM)
                gpio_set_function(outputpin[i].pin, GPIO_FUNC_PWM);
        }
    }

 // Stepper init

    uint32_t pio_offset;

    timer = pio_add_program(pio1, &stepper_timer_program);
    stepper_timer_program_init(pio1, 0, timer, 12.5f); // 10MHz

    irq_set_exclusive_handler(PIO1_IRQ_0, stepper_int_handler);
 //   irq_set_priority(PIO1_IRQ_0, 0);

#ifdef STEP_SINGLE_PIO
  #ifdef STEP_PIN_BASE_X
    pulse_x = pio_add_program(pio0, &step_pulse_program);
    step_pulse_program_init(pio0, 0, pulse_x, STEP_PIN_BASE_X, 1);
  #endif
  #ifdef STEP_PIN_BASE_Y
    pulse_y = pio_add_program(pio0, &step_pulse_program);
    step_pulse_program_init(pio0, 0, pulse_y, STEP_PIN_BASE_Y, 1);
  #endif
  #ifdef STEP_PIN_BASE_Z
    pulse_z = pio_add_program(pio0, &step_pulse_program);
    step_pulse_program_init(pio0, 0, pulse_z, STEP_PIN_BASE_Z, 1);
  #endif
#elif SD_SHIFT_REGISTER
    pio_offset = pio_add_program(pio0, &step_dir_sr4_program);
    step_dir_sr4_program_init(pio0, 0, pio_offset, SD_SR_DATA_PIN, SD_SR_SCK_PIN);

    pio_offset = pio_add_program(pio0, &sr_delay_program);
    sr_delay_program_init(pio0, 1, pio_offset,  11.65f);

    pio_offset= pio_add_program(pio0, &sr_hold_program);
    sr_hold_program_init(pio0, 2, pio_offset, 11.65f);
#else
    pulse = pio_add_program(pio0, &step_pulse_program);
    step_pulse_program_init(pio0, 0, pulse, STEP_PINS_BASE, N_AXIS + N_GANGED);
#endif

#if OUT_SHIFT_REGISTER
    pio_offset = pio_add_program(pio1, &out_sr16_program);
    out_sr16_program_init(pio1, 1, pio_offset, OUT_SR_DATA_PIN, OUT_SR_SCK_PIN);
#endif

#if SDCARD_ENABLE
    sdcard_init();
#endif

#if MPG_MODE == 1
    gpio_init(MPG_MODE_PIN);
#endif

    IOInitDone = settings->version == 21;

    hal.settings_changed(settings);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

#if PPI_ENABLE
    ppi_init();
#endif

    return IOInitDone;
}

#if USB_SERIAL_CDC
static void execute_realtime (uint_fast16_t state)
{
    if(ms_event) {
        ms_event = false;
        usb_execute_realtime(state);
    }
}
#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: grblHAL is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

//    irq_set_exclusive_handler(-1, systick_handler);

    systick_hw->rvr = 999;
    systick_hw->cvr = 0;
    systick_hw->csr = M0PLUS_SYST_CSR_TICKINT_BITS|M0PLUS_SYST_CSR_ENABLE_BITS;

    hal.info = "RP2040";
    hal.driver_version = "220115";
    hal.driver_options = "SDK_" PICO_SDK_VERSION_STRING;
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = 10000000;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
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

    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle.update_rpm = spindleUpdateRPM;
#endif
#if PPI_ENABLE
    hal.spindle.pulse_on = spindlePulseOn;
#endif

    hal.control.get_state = systemGetState;

#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

#ifdef SERIAL2_MOD
    serial2Init(115200);
#endif

#if USB_SERIAL_CDC
    stream_connect(serial_stream = usb_serialInit());
    grbl.on_execute_realtime = execute_realtime;
#else
    stream_connect(serial_stream = serialInit(115200));
#endif

#ifdef I2C_PORT
    I2C_Init();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
#else
    hal.nvs.type = NVS_None;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
#if ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
    hal.signals_cap.reset = Off;
#endif
    hal.driver_cap.spindle_dir = On;
#ifdef SPINDLE_PWM_PIN
    hal.driver_cap.variable_spindle = On;
#endif
    hal.driver_cap.spindle_pwm_invert = On;
#if defined(COOLANT_MIST_PIN) || OUT_SHIFT_REGISTER
    hal.driver_cap.mist_control = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
#ifdef PROBE_PIN
    hal.driver_cap.probe_pull_up = On;
#endif

    uint32_t i;
    input_signal_t *input;
    output_signal_t *output;

    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->id = Input_Aux0 + aux_inputs.n_pins++;
            input->cap.irq_mode = IRQ_Mode_All;
        }
        if(input->group == PinGroup_Limit) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        }
#ifdef SAFETY_DOOR_PIN
        if(input->id == Input_SafetyDoor)
            safety_door = input;
#endif
    }

    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output= &outputpin[i];
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = Output_Aux0 + aux_outputs.n_pins++;
        }
    }

#ifdef HAS_BOARD_INIT
  #if OUT_SHIFT_REGISTER
    board_init(&aux_inputs, &aux_outputs, &out_sr);
  #endif
#endif

    serialRegisterStreams();

#if MPG_MODE == 1
    if(hal.driver_cap.mpg_mode = stream_mpg_register(serialInit(115200), false, NULL))
        protocol_enqueue_rt_command(mpg_enable);
#elif MPG_MODE == 2
    hal.driver_cap.mpg_mode = stream_mpg_register(serialInit(115200), false, keypad_enqueue_keycode);
#elif KEYPAD_ENABLE == 2
    stream_open_instance(0, 115200, keypad_enqueue_keycode);
#endif

#if IOEXPAND_ENABLE
    ioexpand_init();
#endif

#if MODBUS_ENABLE
    modbus_init(serial2Init(115200));
#endif

#if SPINDLE_HUANYANG > 0
    huanyang_init();
#endif

#include "grbl/plugins_init.h"

  //  debounceAlarmPool = alarm_pool_create(DEBOUNCE_ALARM_HW_TIMER, DEBOUNCE_ALARM_MAX_TIMER);

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 9;
}

/* interrupt handlers */

// Main stepper driver
void __not_in_flash_func(stepper_int_handler)(void)
{
    stepper_timer_irq_clear(pio1);

    hal.stepper.interrupt_callback();
}

// Limit debounce callback
static int64_t __not_in_flash_func(limit_debounce_callback)(alarm_id_t id, void *input)
{
    if(((input_signal_t *)input)->debounce) {

        ((input_signal_t *)input)->debounce = false;
        gpio_set_irq_enabled(((input_signal_t *)input)->pin, ((input_signal_t *)input)->invert ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE, true);
    /*
        debounce_pool_t * pool = (debounce_pool_t *)array;

        // Find which pin set this callback and re-enable its IRQ
        for(int i=0; i<DEBOUNCE_ALARM_MAX_TIMER; i++) {
            if(pool[i].id == id) {
                gpio_set_irq_enabled(pool[i].pin, GPIO_IRQ_EDGE_RISE, true);
                break;
            }
        }
        */

        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value)
            hal.limits.interrupt_callback(state);
    }

    return 0;
}

// SR Latch callback - used to delay resetting of pins after they are triggered
static int64_t __not_in_flash_func(srLatch_debounce_callback)(alarm_id_t id, void *input)
{
    if(((input_signal_t *)input)->id == Input_Probe)
        gpio_set_irq_enabled(((input_signal_t *)input)->pin, probe.inverted ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW, true);
    else if(((input_signal_t *)input)->id == Input_SafetyDoor)
        gpio_set_irq_enabled(((input_signal_t *)input)->pin, ((input_signal_t *)input)->invert ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW, true);
 
    /*
    debounce_pool_t * pool = (debounce_pool_t *)array;

    // Find which pin set this callback and re-enable its IRQ
    for(int i=0; i<DEBOUNCE_ALARM_MAX_TIMER; i++) {
        if(pool[i].id == id) {
            gpio_set_irq_enabled(pool[i].pin, pool[i].level, true);
            return 0;;
        }
    }
*/
    return 0;
}

#if PPI_ENABLE

// PPI timer interrupt handler
void PPI_TIMER_IRQHandler (void)
{
    PPI_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    spindle_off();
}

#endif

// GPIO Interrupt handler
// TODO: bypass the Pico library interrupt handler.
void __not_in_flash_func(gpio_int_handler)(uint gpio, uint32_t events)
{
    input_signal_t *input = NULL;
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    do {
        if(inputpin[--i].pin == gpio)
            input = &inputpin[i];
    } while(i && !input);

    if(input) switch(input->group) {

        case PinGroup_Control:

#ifdef SAFETY_DOOR_PIN
            if(input->id == Input_SafetyDoor) {
                gpio_set_irq_enabled(gpio, GPIO_IRQ_ALL, false);
                // If the input is active fire the control interrupt immediately and register an
                // alarm to reenable the interrupt after a short delay. Only after this delay has
                // expired can the safety door signal be set inactive. This is done to avoid stressing
                // the main state-machine.
                if((input->active = !!(events & GPIO_IRQ_LEVEL_HIGH) ^ input->invert)) {
                    hal.control.interrupt_callback(systemGetState());
                    if(!add_alarm_in_ms(SR_LATCH_DEBOUNCE_TEMPO, srLatch_debounce_callback, (void*)input, false))
                        gpio_set_irq_enabled(gpio, GPIO_IRQ_LEVEL_HIGH, true); // Reenable the IRQ in case the alarm wasn't registered.
                } else
                    gpio_set_irq_enabled(gpio, !input->invert ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW, true);
            } else
#endif
            hal.control.interrupt_callback(systemGetState());
            break;
            
#if PROBE_PIN
        case PinGroup_Probe:
            gpio_set_irq_enabled(gpio, GPIO_IRQ_ALL, false);
            // If input is active set the probe signal active immediately and register an
            // alarm to reenable the interrupt after a short delay. Only after this delay has
            // expired can the probe signal be set inactive.
            if((probe.triggered = !!(events & GPIO_IRQ_LEVEL_HIGH) ^ probe.inverted)) {
                if(!add_alarm_in_ms(SR_LATCH_DEBOUNCE_TEMPO, srLatch_debounce_callback, (void*)input, false))
                    gpio_set_irq_enabled(gpio, probe.inverted ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW, true); // Reenable the IRQ in case the alarm wasn't registered.
            } else
                gpio_set_irq_enabled(gpio, probe.inverted ? GPIO_IRQ_LEVEL_LOW : GPIO_IRQ_LEVEL_HIGH, true);
            break;
#endif

        case PinGroup_Limit:
            {
                // If debounce is enabled register an alarm to reenable the IRQ after the debounce delay has expired.
                // If the input is still active when the delay expires the limits interrupt will be fired.
                if(hal.driver_cap.software_debounce && add_alarm_in_ms(LIMIT_DEBOUNCE_TEMPO, limit_debounce_callback, (void*)input, true)) {
                    input->debounce = true;
                    gpio_set_irq_enabled(gpio, GPIO_IRQ_ALL, false); // Disable the pin IRQ for the duration of the debounce delay.
                } else
                    hal.limits.interrupt_callback(limitsGetState());
            }
            break;

#ifdef HAS_IOPORTS
        case PinGroup_AuxInput:
            ioports_event(input);
            break;
#endif
#if I2C_STROBE_ENABLE
        case PinGroup_Keypad:
            if(i2c_strobe.callback)
                i2c_strobe.callback(0, DIGITAL_IN(input->bit) == 0);
            break;
#endif
#if MPG_MODE == 1
        case PinGroup_MPG:
            pinEnableIRQ(input, IRQ_Mode_None);
            protocol_enqueue_rt_command(mpg_select);
            break;
#endif
        default:
            break;
    }
}

// Interrupt handler for 1 ms interval timer
void __not_in_flash_func(isr_systick)(void)
{
    ms_event = true;
    elapsed_ticks++;

#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif
}