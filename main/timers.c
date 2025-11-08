/*

  timers.c - driver code for ESP32 processors

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

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

typedef struct {
	uint32_t group;
	uint32_t index;
    timer_resolution_t resolution;
    bool claimed;
    timer_cfg_t cfg;
    timer_cap_t cap;
    uint32_t freq_hz;
    uint32_t timebase;
    void (*isr)(void * arg);
} dtimer_t;

IRAM_ATTR static void timer01_isr (void *arg)
{
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG0.int_clr_timers.t1_int_clr = 1;
#else
    TIMERG0.int_clr_timers.t1 = 1;
#endif
    ((dtimer_t *)arg)->cfg.timeout_callback(((dtimer_t *)arg)->cfg.context);
}

IRAM_ATTR static void timer10_isr (void *arg)
{
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG1.int_clr_timers.t0_int_clr = 1;
#else
    TIMERG1.int_clr_timers.t0 = 1;
#endif
    ((dtimer_t *)arg)->cfg.timeout_callback(((dtimer_t *)arg)->cfg.context);
}

IRAM_ATTR static void timer11_isr (void *arg)
{
#if CONFIG_IDF_TARGET_ESP32S3
    TIMERG1.int_clr_timers.t1_int_clr = 1;
#else
    TIMERG1.int_clr_timers.t1 = 1;
#endif
    ((dtimer_t *)arg)->cfg.timeout_callback(((dtimer_t *)arg)->cfg.context);
}

static dtimer_t timers[] = {
	{ .group = 0, .index = 1, .freq_hz = 80000000, .isr = timer01_isr },
	{ .group = 1, .index = 0, .freq_hz = 80000000, .isr = timer10_isr },
	{ .group = 1, .index = 1, .freq_hz = 80000000, .isr = timer11_isr }
};

hal_timer_t timerClaim (timer_cap_t cap, uint32_t timebase)
{
    hal_timer_t t;
    uint_fast8_t idx, n_timers = sizeof(timers) / sizeof(dtimer_t);

    for(idx = 0; idx < n_timers; idx++) {
        if((t = timers[idx].claimed ? NULL : &timers[idx])) {
        	timers[idx].claimed = true;
        	timers[idx].timebase = timebase;
        	break;
        }
    }

    return t;
}

bool timerCfg (hal_timer_t timer, timer_cfg_t *cfg)
{
    bool ok;

    if((ok = timer != NULL)) {

    	dtimer_t *dtimer = (dtimer_t *)timer;

        memcpy(&dtimer->cfg, cfg, sizeof(timer_cfg_t));

        timer_config_t timerConfig = {
            .counter_dir = TIMER_COUNT_UP,
            .counter_en  = TIMER_PAUSE,
            .alarm_en    = TIMER_ALARM_EN,
            .intr_type   = TIMER_INTR_LEVEL
        };

        timerConfig.auto_reload = !cfg->single_shot;
        timerConfig.divider = ((dtimer->freq_hz / 1000) * dtimer->timebase) / 1000000;

        timer_init(dtimer->group, dtimer->index, &timerConfig);
        timer_set_counter_value(dtimer->group, dtimer->index, 0ULL);
        timer_isr_register(dtimer->group, dtimer->index, dtimer->isr, timer, ESP_INTR_FLAG_IRAM, NULL);
        timer_enable_intr(dtimer->group, dtimer->index);
    }

    return ok;
}

bool timerStart (hal_timer_t timer, uint32_t period)
{
    dtimer_t *dtimer = (dtimer_t *)timer;

    if(dtimer->cfg.single_shot || period != dtimer->cfg.period) {

        ((dtimer_t *)timer)->cfg.period = period;

        timer_set_counter_value(dtimer->group, dtimer->index, 0x00000000ULL);
        timer_set_alarm_value(dtimer->group, dtimer->index, (uint64_t)period);
        timer_start(dtimer->group, dtimer->index);
    #if CONFIG_IDF_TARGET_ESP32S3
        TIMERG0.hw_timer[dtimer->index].config.tn_alarm_en = TIMER_ALARM_EN;
    #else
        TIMERG0.hw_timer[dtimer->index].config.alarm_en = TIMER_ALARM_EN;
    #endif
    }

    return true;
}

bool timerStop (hal_timer_t timer)
{
    ((dtimer_t *)timer)->cfg.period = 0;
	timer_pause(((dtimer_t *)timer)->group, ((dtimer_t *)timer)->index);

    return true;
}
