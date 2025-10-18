/*
  spi.c - SPI support for SD card & Trinamic plugins

  Part of grblHAL driver for ESP32

  Copyright (c) 2020-2025 Terje Io

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

#include "driver.h"

#if SPI_ENABLE

#include "driver/sdspi_host.h"
#include "driver/spi_master.h"

static spi_device_handle_t handle = NULL;

bool spi_bus_init (spi_host_device_t *host)
{
    static const periph_pin_t sck = {
        .function = Output_SPICLK,
        .group = PinGroup_SPI,
        .pin = SPI_SCK_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t sdo = {
        .function = Output_MOSI,
        .group = PinGroup_SPI,
        .pin = SPI_MOSI_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    static const periph_pin_t sdi = {
        .function = Input_MISO,
        .group = PinGroup_SPI,
        .pin = SPI_MISO_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    static spi_host_device_t host_id = (spi_host_device_t)99;

    if(host_id == (spi_host_device_t)99) {

		spi_common_dma_t dma_ch;
        spi_bus_config_t bus_config = {
            .mosi_io_num     = SPI_MOSI_PIN,
            .miso_io_num     = SPI_MISO_PIN,
            .sclk_io_num     = SPI_SCK_PIN,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .flags           = SPICOMMON_BUSFLAG_MASTER,
            .intr_flags      = ESP_INTR_FLAG_IRAM
        };

#ifdef CONFIG_IDF_TARGET_ESP32S3
		dma_ch = SPI_DMA_CH_AUTO;
        host_id = SDSPI_DEFAULT_HOST;
#else
		dma_ch = SPI_DMA_CH1;
  #if SPI_SCK_PIN == GPIO_NUM_14
        host_id = SPI2_HOST;
  #elif SPI_SCK_PIN == GPIO_NUM_18
        host_id = SPI3_HOST;
  #else
        host_id = SDSPI_DEFAULT_HOST;
  #endif
#endif

		if(spi_bus_initialize(host_id, &bus_config, dma_ch) == ESP_OK) {
			hal.periph_port.register_pin(&sck);
			hal.periph_port.register_pin(&sdo);
			hal.periph_port.register_pin(&sdi);
		} else
			host_id = (spi_host_device_t)99;
    }

    *host = host_id;

    return host_id != (spi_host_device_t)99;
}

void spi_init (void)
{
    static bool init = false;

    if(!init) {

    	spi_host_device_t host;

    	if((init = spi_bus_init(&host))) {

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

esp_err_t spi_bus (bool aquire)
{
    if(aquire)
        return spi_device_acquire_bus(handle, portMAX_DELAY);
    else
        spi_device_release_bus(handle);

    return ESP_OK;
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
