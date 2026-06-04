/*
  ooznest_cnc.c - driver code for ESP32
  Part of grblHAL
  Copyright (c) 2026 Ooznest Ltd.
*/

#include "driver.h"

#if defined(HAS_BOARD_INIT) && defined(BOARD_OOZNEST_CNC)

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/rmt.h"

#include "i2c.h"
#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/settings.h"
#include "grbl/task.h"
#include "grbl/state_machine.h"
#include "grbl/protocol.h"
#include "grbl/system.h"
#include "grbl/report.h"

#include "sdcard/sdcard.h"

// TODO: Identify and implement the correct 4-channel I2C DAC for current control.
// For now, this is a placeholder based on the original Ooznest board's MCP4725.

typedef struct {
    float current; // mA
} ooznest_motor_settings_t;

typedef struct {
    ooznest_motor_settings_t driver[N_AXIS]; // 3 axes (X, Y, Z)
} ooznest_settings_t;

static ooznest_settings_t mks;
static nvs_address_t nvs_address;
static settings_changed_ptr settings_changed;
static on_realtime_report_ptr prev_realtime_report;

static struct {
    float current;
    uint_fast8_t axis;
} current_iterator;

#if I2C_ENABLE
static float last_vbus_v = 0.0f;
static float last_current_a = 0.0f;
#endif

typedef enum {
    Power_On = 0,
    Power_Alarm,
    Power_Lost
} power_state_t;

static power_state_t power_state = Power_On;
static bool vin_was_ok = false;
static bool power_lost_during_motion = false;

static bool ooznest_is_setting_available (const setting_detail_t *setting, uint_fast16_t subgroup)
{
    return true; // All axes settings available for now
}

static void ooznest_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&mks, sizeof(ooznest_settings_t), true);
}

static void set_motor_current (uint8_t motor_idx, float current)
{
    if (motor_idx >= 4)
        return;

#if I2C_ENABLE
    // MCP4728 DAC channel mapping for Ooznest CNC hardware:
    // Ch 0: X, Ch 1: Y1, Ch 2: Y2, Ch 3: Z
    uint32_t dac_channel = motor_idx;

    // MCP4728 I2C address is usually 0x60
    static const uint8_t dac_addr = 0x60 << 1;

    uint8_t data[3];
    uint32_t v = (uint32_t)(current * 5000.0f * .22f * (4096.0f / 3300.0f));
    if (v > 4095) v = 4095;

    // Single Write Command for MCP4728
    // Byte 1: 0 1 0 0 0 DAC1 DAC0 UDAC (=1 for immediate update)
    data[0] = 0x40 | ((dac_channel & 0x03) << 1) | 0x01;
    // Byte 2: VREF(=0) PD1(=0) PD0(=0) Gx(=0) D11 D10 D9 D8
    data[1] = (v >> 8) & 0x0F;
    // Byte 3: D7 D6 D5 D4 D3 D2 D1 D0
    data[2] = v & 0xFF;

    if (i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dac_addr | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, data[0], true);
        i2c_master_write_byte(cmd, data[1], true);
        i2c_master_write_byte(cmd, data[2], true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, 50 / portTICK_PERIOD_MS);
        (void)err;
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);
    }
#endif
}

static void motor_iterator_callback (motor_map_t motor)
{
    if (motor.axis == current_iterator.axis) {
        set_motor_current(motor.id, current_iterator.current);
    }
}

static void set_current (uint_fast8_t axis, float current)
{
    if (axis >= N_AXIS)
        return;

    if (hal.stepper.motor_iterator) {
        current_iterator.axis = axis;
        current_iterator.current = current;
        hal.stepper.motor_iterator(motor_iterator_callback);
    } else {
        // Fallback for very early initialization if iterator is not yet available
        uint8_t motor_idx = axis == 2 ? 3 : axis;
        set_motor_current(motor_idx, current);
        if (axis == 1) { // Default Y ganging fallback
            set_motor_current(2, current);
        }
    }
}

static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    status_code_t status = Status_OK;
    uint_fast8_t idx;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisStepperCurrent:
            if (idx < N_AXIS) {
                mks.driver[idx].current = value;
                set_current(idx, value);
            } else status = Status_InvalidStatement;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static float get_axis_setting (setting_id_t setting)
{
    float value = 0.0f;
    uint_fast8_t idx;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisStepperCurrent:
            if (idx < N_AXIS)
                value = mks.driver[idx].current;
            break;

        default:
            break;
    }

    return value;
}

static const setting_detail_t ooznest_settings[] = {
    { Setting_AxisStepperCurrent, Group_Axis0, "-axis motor current", "A", Format_Decimal, "0.0#", "0", "3.0", Setting_NonCoreFn, (void *)set_axis_setting, (void *)get_axis_setting, ooznest_is_setting_available, { .subgroups = On, .increment = 1 } }
};

static void ooznest_settings_restore (void)
{
    uint_fast8_t idx = N_AXIS;
    do {
        mks.driver[--idx].current = 0.6f;
    } while(idx);
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&mks, sizeof(ooznest_settings_t), true);
}

static void ooznest_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&mks, nvs_address, sizeof(ooznest_settings_t), true) != NVS_TransferResult_OK)
        ooznest_settings_restore();

    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        set_current(idx, mks.driver[idx].current);
    } while(idx);

    ioport_setting_changed(Setting_ControlInvertMask);
}


static void ooznest_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    ioport_setting_changed(Setting_ControlInvertMask);
    if(settings_changed)
        settings_changed(settings, changed);

}

static setting_details_t setting_details = {
    .settings = ooznest_settings,
    .n_settings = sizeof(ooznest_settings) / sizeof(setting_detail_t),
    .load = ooznest_settings_load,
    .save = ooznest_settings_save,
    .restore = ooznest_settings_restore
};

#if I2C_ENABLE
static void raise_power_alarm (void *data);
static void check_power_restored (void *data);

// INA219 Polling Task (Non-blocking grblHAL task)
static void ina219_task (void *pvParameters)
{
    static uint32_t last_poll = 0;

    if (hal.get_elapsed_ticks() - last_poll < 500) return;
    last_poll = hal.get_elapsed_ticks();

    const uint8_t ina_addr = 0x40 << 1;

    if (i2cBusy != NULL && xSemaphoreTake(i2cBusy, 0) == pdTRUE) {

        uint8_t bus_data[2] = {0};
        uint8_t shunt_data[2] = {0};

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ina_addr | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x02, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ina_addr | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &bus_data[0], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &bus_data[1], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        esp_err_t err_v = i2c_master_cmd_begin(I2C_PORT, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ina_addr | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x01, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ina_addr | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &shunt_data[0], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &shunt_data[1], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        esp_err_t err_i = i2c_master_cmd_begin(I2C_PORT, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);

        if (err_v == ESP_OK && err_i == ESP_OK) {
            uint16_t bus_reg = (bus_data[0] << 8) | bus_data[1];
            int16_t shunt_reg = (shunt_data[0] << 8) | shunt_data[1];

            float vbus_v = (bus_reg >> 3) * 0.004f;
            float shunt_mv = shunt_reg * 0.01f;
            float current_a = (shunt_mv / 1000.0f) / 0.03f;

            last_vbus_v = vbus_v;
            last_current_a = current_a;

            if(vbus_v > 19.0f) {
                vin_was_ok = true;
            } else if(vin_was_ok && vbus_v < 18.0f && power_state == Power_On) {
                vin_was_ok = false;
                power_state = Power_Alarm;
                power_lost_during_motion = !!(state_get() & (STATE_CYCLE|STATE_JOG|STATE_HOMING));
                protocol_enqueue_foreground_task(raise_power_alarm, NULL);
            }
        }
    }
}

static void raise_power_alarm (void *data)
{
    if(power_state == Power_Alarm) {
        system_raise_alarm(Alarm_MotorFault);
        task_add_delayed(check_power_restored, NULL, 250);
    }

    power_state = Power_Lost;
}

static void check_power_restored (void *data)
{
    if(power_state != Power_Lost)
        return;

    if(last_vbus_v > 19.0f) {

        power_state = Power_On;
        vin_was_ok = true;

        if(power_lost_during_motion) {
            power_lost_during_motion = false;
            report_message("Motor power restored - check workpiece, position may be lost", Message_Info);
        } else {
            report_message("Motor power restored", Message_Info);

            if(hal.stepper.status)
                hal.stepper.status(true);
        }

    } else
        task_add_delayed(check_power_restored, NULL, 250);
}
#endif

// Realtime loop report intercept
static void ooznest_on_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if (prev_realtime_report)
        prev_realtime_report(stream_write, report);

#if I2C_ENABLE // append INA reading INA219:VOLTS,AMPS
    stream_write("|INA219:");
    stream_write(ftoa(last_vbus_v, 2));
    stream_write(",");
    stream_write(ftoa(last_current_a, 2));
#endif
}

#if !STATUS_LIGHT_ENABLE
// --- LED Status Control ---

#define LED_OFF     (rgb_color_t){ .R = 0, .G = 0, .B = 0 }
#define LED_RED     (rgb_color_t){ .R = 255, .G = 0, .B = 0 }
#define LED_GREEN   (rgb_color_t){ .R = 0, .G = 255, .B = 0 }
#define LED_BLUE    (rgb_color_t){ .R = 0, .G = 0, .B = 255 }
#define LED_YELLOW  (rgb_color_t){ .R = 255, .G = 255, .B = 0 }
#define LED_WHITE   (rgb_color_t){ .R = 255, .G = 255, .B = 255 }
#define LED_ORANGE  (rgb_color_t){ .R = 255, .G = 102, .B = 0 }
#define LED_TEAL    (rgb_color_t){ .R = 0, .G = 128, .B = 128 }

static on_state_change_ptr on_state_change;
static on_program_completed_ptr on_program_completed;

static void rgb_set_all (rgb_color_t color)
{
    if(!hal.rgb0.out) return;
    for(uint16_t device = 0; device < hal.rgb0.num_devices; device++)
        hal.rgb0.out(device, color);
    if(hal.rgb0.write)
        hal.rgb0.write();
}

static void rgb_set_idle (void)
{
    if(!hal.rgb0.out) return;
    uint16_t num = hal.rgb0.num_devices;
    hal.rgb0.out(0, LED_GREEN);
    if(num > 1) hal.rgb0.out(1, LED_ORANGE);
    if(num > 2) hal.rgb0.out(2, LED_TEAL);
    for(uint16_t device = 3; device < num; device++) {
        float t = (float)(device - 3) / (num > 3 ? num - 3 : 1);
        hal.rgb0.out(device, (rgb_color_t){
            .R = (uint8_t)(255 + (0 - 255) * t),
            .G = (uint8_t)(102 + (128 - 102) * t),
            .B = (uint8_t)(0 + (128 - 0) * t)
        });
    }
    if(hal.rgb0.write)
        hal.rgb0.write();
}

// Alarm: running chase left to right
static void ooznest_led_alarm (void *data)
{
    if(!(state_get() & (STATE_ALARM | STATE_ESTOP)))
        return;

    static uint16_t pos = 0;
    uint16_t num = hal.rgb0.num_devices;

    for(uint16_t device = 0; device < num; device++) {
        uint16_t dist = pos > device ? pos - device : device - pos;
        if(dist == 0)
            hal.rgb0.out(device, LED_RED);
        else if(dist == 1)
            hal.rgb0.out(device, (rgb_color_t){ .R = 160, .G = 0, .B = 0 });
        else if(dist == 2)
            hal.rgb0.out(device, (rgb_color_t){ .R = 80, .G = 0, .B = 0 });
        else
            hal.rgb0.out(device, (rgb_color_t){ .R = 16, .G = 0, .B = 0 });
    }
    if(hal.rgb0.write)
        hal.rgb0.write();

    if(++pos >= num) pos = 0;

    task_add_delayed(ooznest_led_alarm, NULL, 50);
}

// Homing: expanding ring from center outward, then reset
static void ooznest_led_homing (void *data)
{
    if(state_get() != STATE_HOMING)
        return;

    static uint16_t radius = 0;
    static int8_t dir = 1;
    uint16_t num = hal.rgb0.num_devices;
    uint16_t center = num / 2;

    for(uint16_t device = 0; device < num; device++) {
        uint16_t dist = device < center ? center - device : device - center;
        hal.rgb0.out(device, dist <= radius ? LED_BLUE : LED_OFF);
    }
    if(hal.rgb0.write)
        hal.rgb0.write();

    if(dir > 0) {
        if(++radius >= center) dir = -1;
    } else {
        if(radius-- == 0) dir = 1;
    }

    task_add_delayed(ooznest_led_homing, NULL, 60);
}

static void ooznest_update_leds (sys_state_t state)
{
    switch(state) {
        case STATE_IDLE:
            rgb_set_idle();
            break;

        case STATE_ALARM:
        case STATE_ESTOP:
            task_add_immediate(ooznest_led_alarm, NULL);
            break;

        case STATE_HOMING:
        case STATE_CHECK_MODE:
            task_add_immediate(ooznest_led_homing, NULL);
            break;

        case STATE_SAFETY_DOOR:
            rgb_set_all(LED_YELLOW);
            break;

        case STATE_CYCLE:
        case STATE_JOG:
        case STATE_HOLD:
        case STATE_SLEEP:
        case STATE_TOOL_CHANGE:
        default:
            rgb_set_all(LED_WHITE);
            break;
    }
}

static void ooznest_on_state_change (sys_state_t state)
{
    ooznest_update_leds(state);

    if(on_state_change)
        on_state_change(state);
}

static void ooznest_job_completed (void *data)
{
    static uint8_t flash_count = 0;

    if(flash_count >= 6) {
        flash_count = 0;
        ooznest_update_leds(state_get());
        return;
    }

    rgb_set_all(flash_count++ % 2 ? LED_OFF : LED_GREEN);
    if(hal.rgb0.write)
        hal.rgb0.write();

    task_add_delayed(ooznest_job_completed, NULL, 150);
}

static void ooznest_on_program_completed (program_flow_t program_flow, bool check_mode)
{
    if(!check_mode)
        task_add_immediate(ooznest_job_completed, NULL);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void ooznest_init_leds (void *data)
{
    ooznest_update_leds(state_get());
}
#endif

void board_init (void)
{
    #if I2C_ENABLE
    i2c_start();
    #endif

    nvs_address = nvs_alloc(sizeof(ooznest_settings_t));
    settings_register(&setting_details);

    if (hal.settings_changed != ooznest_settings_changed) {
        settings_changed = hal.settings_changed;
        hal.settings_changed = ooznest_settings_changed;
    }

    if (grbl.on_realtime_report != ooznest_on_realtime_report) {
         prev_realtime_report = grbl.on_realtime_report;
         grbl.on_realtime_report = ooznest_on_realtime_report;
    }

    hal.signals_cap.motor_warning = Off;
    hal.driver_cap.probe2 = On;

#if !STATUS_LIGHT_ENABLE
    on_state_change = grbl.on_state_change;
    grbl.on_state_change = ooznest_on_state_change;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = ooznest_on_program_completed;

    task_run_on_startup(ooznest_init_leds, NULL);
#endif

    #if I2C_ENABLE
      // POST: Check for I2C DAC
      if (i2cBusy != NULL && xSemaphoreTake(i2cBusy, 100 / portTICK_PERIOD_MS) == pdTRUE) {
          i2c_cmd_handle_t cmd = i2c_cmd_link_create();
          i2c_master_start(cmd);
          i2c_master_write_byte(cmd, (0x60 << 1) | I2C_MASTER_WRITE, true);
          i2c_master_stop(cmd);
          esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, 50 / portTICK_PERIOD_MS);
          i2c_cmd_link_delete(cmd);
          xSemaphoreGive(i2cBusy);

          if (err == ESP_OK) {
              ESP_LOGI("Ooznest", "Hardware Check: MCP4728 DAC detected!");
          } else {
              ESP_LOGE("Ooznest", "Hardware Check: MCP4728 DAC NOT found (I2C Error %d)", err);
          }

          // Start INA219 task natively in grblHAL
          task_add_systick(ina219_task, NULL);
      }
    #endif

    #if ETHERNET_ENABLE

      gpio_config_t gpioConfig = {
          .pin_bit_mask = 1ULL << INPUT_GPIO_CS,
          .mode = GPIO_MODE_INPUT,
          .pull_up_en = GPIO_PULLUP_DISABLE,
          .pull_down_en = GPIO_PULLDOWN_ENABLE,
          .intr_type = GPIO_INTR_DISABLE
      };

      gpio_config(&gpioConfig);

      hal.driver_cap.ethernet = !!DIGITAL_IN(INPUT_GPIO_CS);

    #endif
  }

#endif // BOARD_OOZNEST_CNC
