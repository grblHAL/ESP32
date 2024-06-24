/*
  spi.c - SPI support for SD card & Trinamic plugins

  Part of grblHAL driver for ESP32

  Copyright (c) 2020-2024 Terje Io

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

#include "driver.h"

#if SPI_ENABLE

#include "driver/sdspi_host.h"
#include "driver/spi_master.h"

static spi_device_handle_t handle = NULL;

void spi_init (void)
{
    static bool init = false;

    if(!init) {

        spi_bus_config_t bus_config = {
            .mosi_io_num     = PIN_NUM_MOSI,
            .miso_io_num     = PIN_NUM_MISO,
            .sclk_io_num     = PIN_NUM_CLK,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = 40,
            .flags           = SPICOMMON_BUSFLAG_MASTER,
            .intr_flags      = ESP_INTR_FLAG_IRAM
        };

#if CONFIG_IDF_TARGET_ESP32S3
        if((init = spi_bus_initialize(SDSPI_DEFAULT_HOST, &bus_config, SPI_DMA_CH_AUTO) == ESP_OK)) {
#else
  #if PIN_NUM_CLK == GPIO_NUM_14
        if((init = spi_bus_initialize(SPI2_HOST, &bus_config, 1) == ESP_OK)) { // 1 = SPI_DMA_CH1
  #elif PIN_NUM_CLK == GPIO_NUM_18
        if((init = spi_bus_initialize(SPI3_HOST, &bus_config, 1) == ESP_OK)) {
  #else
        if((init = spi_bus_initialize(SDSPI_DEFAULT_HOST, &bus_config, 1) == ESP_OK)) {
  #endif
#endif
            spi_device_interface_config_t devcfg = {
                .clock_speed_hz = 1000000,
                .mode = 0,          //SPI mode 0
                .spics_io_num = -1,
                .queue_size = 1,
            //    .flags = SPI_DEVICE_POSITIVE_CS,
            //   .pre_cb = cs_high,
            //   .post_cb = cs_low,
                .input_delay_ns = 0  //the EEPROM output the data half a SPI clock behind.
            };

            spi_bus_add_device(SPI2_HOST, &devcfg, &handle);

            static const periph_pin_t sck = {
                .function = Output_SPICLK,
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
        }
    }
}

// set the SSI speed to the max setting
void spi_set_max_speed (void)
{

}

uint32_t spi_set_speed (uint32_t prescaler)
{
    uint32_t cur = 0;

    return cur;
}

uint8_t spi_get_byte (void)
{
    spi_transaction_t t = {
        .cmd = 0,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
        .user = NULL,
    };

    return spi_device_polling_transmit(handle, &t) == ESP_OK ? t.rx_data[0] : 0xFF;
}

uint8_t spi_put_byte (uint8_t byte)
{ 
//    esp_err_t err;

//    err = spi_device_acquire_bus(handle, portMAX_DELAY);
//    if (err != ESP_OK) return err;

    spi_transaction_t t = {
        .cmd = 0,
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA, // |SPI_TRANS_MODE_OCT, fails on earlier version of the IDF
        .tx_data[0] = byte,
        .user = NULL,
    };

    return spi_device_polling_transmit(handle, &t) == ESP_OK ? 0 : 0xFF;
}

#endif

