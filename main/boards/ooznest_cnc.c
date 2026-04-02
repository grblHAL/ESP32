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

#include "sdcard/sdcard.h"

// TODO: Identify and implement the correct 4-channel I2C DAC for current control.
// For now, this is a placeholder based on the original Ooznest board's MCP4725.

typedef struct {
    float current; // mA
} ooznest_motor_settings_t;

typedef struct {
    ooznest_motor_settings_t driver[N_AXIS];
} ooznest_settings_t;

static ooznest_settings_t mks;
static nvs_address_t nvs_address;
static settings_changed_ptr settings_changed;
static on_realtime_report_ptr prev_realtime_report;

#if I2C_ENABLE
static float last_vbus_v = 0.0f;
static float last_current_a = 0.0f;
#endif

static bool ooznest_is_setting_available (const setting_detail_t *setting, uint_fast16_t subgroup)
{
    return true; // All axes settings available for now
}

static void ooznest_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&mks, sizeof(ooznest_settings_t), true);
}

static void set_current (uint_fast8_t axis, float current)
{
    if (axis >= N_AXIS)
        return;

#if I2C_ENABLE
    // MCP4728 I2C address is usually 0x60
    static const uint8_t dac_addr = 0x60 << 1;

    uint8_t data[3];
    uint32_t v = (uint32_t)(current * 5000.0f * .22f * (4096.0f / 3300.0f));
    if (v > 4095) v = 4095;

    // Single Write Command for MCP4728
    // Byte 1: 0 1 0 0 0 DAC1 DAC0 UDAC (=1 for immediate update)
    data[0] = 0x40 | ((axis & 0x03) << 1) | 0x01;
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

static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    uint_fast8_t idx;
    status_code_t status = Status_OK;

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

    // settings.control_invert.motor_fault = settings.control_invert.motor_warning = On;
    ioport_setting_changed(Setting_ControlInvertMask);
}


static void ooznest_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    // settings->control_invert.motor_fault = settings->control_invert.motor_warning = On;
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
// INA219 Polling Task (Non-blocking grblHAL task)
static void ina219_task (void *pvParameters)
{
    static uint32_t last_poll = 0;

    // Poll every 500ms
    if (hal.get_elapsed_ticks() - last_poll < 500) return;
    last_poll = hal.get_elapsed_ticks();

    const uint8_t ina_addr = 0x40 << 1;

    // Use zero delay for semaphore to avoid blocking the main grbl loop
    if (i2cBusy != NULL && xSemaphoreTake(i2cBusy, 0) == pdTRUE) {

        uint8_t bus_data[2] = {0};
        uint8_t shunt_data[2] = {0};

        // Read Bus Voltage Register (0x02)
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

        // Read Shunt Voltage Register (0x01)
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
            float current_a = (shunt_mv / 1000.0f) / 0.03f; // R = 0.03 ohm

            last_vbus_v = vbus_v;
            last_current_a = current_a;
        }
    }
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

static void set_leds (void *data)
{
    if(hal.rgb0.out) {
        hal.rgb0.out(0, (rgb_color_t){ .R = 0, .G = 255, .B = 0 }); // Green for OK
        if(hal.rgb0.write)
            hal.rgb0.write();
    }
}

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
    hal.driver_cap.ethernet = On;
    hal.driver_cap.probe2 = On;

    task_add_immediate(set_leds, NULL);

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
  }

#endif // BOARD_OOZNEST_CNC
