/*
  neopixel_spi.c - SPI support for Neopixels, non-blocking

  Part of grblHAL driver for ESP32

  Copyright (c) 2024-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#ifdef NEOPIXEL_SPI

#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
//#include "hal/spi_ll.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define NP_DMA_CH SPI_DMA_CH_AUTO
  #define NP_HOST SPI3_HOST
#else
  #define NP_DMA_CH SPI_DMA_CH2
  #if SPI_SCK_PIN == GPIO_NUM_14
    #define NP_HOST SPI3_HOST
  #elif SPI_SCK_PIN == GPIO_NUM_18
    #define NP_HOST SPI2_HOST
  #else
    #define NP_HOST SPI3_HOST
  #endif
#endif

static bool busy = false;
static neopixel_cfg_t neopixel = { .intensity = 255 };
static spi_device_handle_t handle = NULL;

static settings_changed_ptr settings_changed;

static inline void _write (void)
{
    spi_transaction_t t = {
        .cmd = 0,
        .length = neopixel.num_bytes << 3,
        .tx_buffer = neopixel.leds
    };

    if(!busy)
        busy = spi_device_queue_trans(handle, &t, portMAX_DELAY) == ESP_OK;
}

void neopixels_write (void)
{
    if(neopixel.num_leds > 1)
        _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        rgb_3bpp_pack(&neopixel.leds[device * 9], color, mask, neopixel.intensity);

        if(neopixel.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != intensity) {

        neopixel.intensity = intensity;

        if(neopixel.num_leds) {

            uint_fast16_t device = neopixel.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_3bpp_unpack(& neopixel.leds[device * 9], prev);
                neopixel_out(device, color);
            } while(device);

            if(neopixel.num_leds != 1)
                _write();
        }
    }

    return prev;
}

void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(neopixel.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip.length0) {

        hal.rgb0.num_devices = settings->rgb_strip.length0;

        if(neopixel.leds) {
            free(neopixel.leds);
            neopixel.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
            neopixel.num_bytes = hal.rgb0.num_devices * 9 + 24;
            if((neopixel.leds = calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb0.num_devices = 0;
        }

        neopixel.num_leds = hal.rgb0.num_devices;

        rgb_clear(&hal.rgb0);
    }

    if(settings_changed)
        settings_changed(settings, changed);
}

void post_cb (spi_transaction_t *t)
{
    busy = false;
}

static inline void spi_ll_set_mosi_free_level(spi_dev_t *hw, bool level)
{
    hw->ctrl.d_pol = level;
}

static inline void spi_ll_apply_config(spi_dev_t *hw)
{
    hw->cmd.update = 1;
    while (hw->cmd.update);
}

void neopixel_spi_init (void)
{
    static bool init = false;

    if(!init) {

        spi_bus_config_t bus_config = {
            .mosi_io_num     = LED_PIN,
            .miso_io_num     = -1,
            .sclk_io_num     = -1,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = 2024, // 200 LEDs max
            .flags           = SPICOMMON_BUSFLAG_MASTER,
            .intr_flags      = ESP_INTR_FLAG_IRAM
        };

        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 2450000,
            .mode           = 0,
            .spics_io_num   = -1,
            .queue_size     = 1,
            .post_cb        = post_cb
        };

        if((init = spi_bus_initialize(NP_HOST, &bus_config, NP_DMA_CH) == ESP_OK &&
                    spi_bus_add_device(NP_HOST, &devcfg, &handle) == ESP_OK)) {

            static const periph_pin_t sdi = {
                .function = Output_MOSI,
                .group = PinGroup_SPI,
                .pin = LED_PIN,
                .mode = { .mask = PINMODE_NONE },
                .description = "Neopixels"
            };

            if(NP_HOST == SPI2_HOST) {
                spi_ll_set_mosi_free_level(&GPSPI2, 0);
                spi_ll_apply_config(&GPSPI2);
            } else {
                spi_ll_set_mosi_free_level(&GPSPI3, 0);
                spi_ll_apply_config(&GPSPI3);
            }

            hal.periph_port.register_pin(&sdi);

            hal.rgb0.out = neopixel_out;
            hal.rgb0.out_masked = neopixel_out_masked;
            hal.rgb0.set_intensity = neopixels_set_intensity;
            hal.rgb0.write = neopixels_write;
            hal.rgb0.flags = (rgb_properties_t){ .is_strip = On };
            hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

            settings_changed = hal.settings_changed;
            hal.settings_changed = onSettingsChanged;
        }
    }
}

#endif // NEOPIXEL_SPI
