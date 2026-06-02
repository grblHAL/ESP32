/*
  neopixel_spi.c - SPI support for Neopixels, non-blocking

  TODO: use I2S interface for more precise timing?

  Part of grblHAL driver for STM32F7xx

  Copyright (c) 2024-2025 Terje Io

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

// https://github.com/adafruit/Adafruit_NeoPixel/blob/master/esp.c

#include "driver.h"

#ifdef NEOPIXEL_RMT

#include "hal/rmt_ll.h"

static neopixel_cfg_t neopixel = { .intensity = 255 };
static settings_changed_ptr settings_changed;

#if CONFIG_IDF_TARGET_ESP32S3
#if N_ABC_MOTORS && X_STEP_PIN < 64
#error "NEOPIXEL_RMT is not usable in this configuration"
#endif
static rmt_config_t neo_config = RMT_DEFAULT_CONFIG_TX(LED_PIN, 3);
#else
#if N_ABC_MOTORS > 4 && X_STEP_PIN < 64
#error "NEOPIXEL_RMT is not usable in this configuration"
#endif
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

void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
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

        rgb_clear(&hal.rgb0);
    }

    if(settings_changed)
        settings_changed(settings, changed);
}

void neopixel_rmt_init (void)
{
    static bool init = false;

    if(!init) {

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
        hal.rgb0.flags = (rgb_properties_t){ .is_blocking = On, .is_strip = On };
        hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

        const periph_pin_t neopixels = {
            .function = Output_LED_Adressable,
            .group = PinGroup_LED,
            .pin = LED_PIN,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        hal.periph_port.register_pin(&neopixels);

        settings_changed = hal.settings_changed;
        hal.settings_changed = onSettingsChanged;

        init = true;
    }
}

#endif // NEOPIXEL_RMT
