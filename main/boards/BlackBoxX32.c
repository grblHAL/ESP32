/*
  BlackBoxX32.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2023 Terje Io

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

#if defined(BOARD_BLACKBOX_X32) && (N_AUTO_SQUARED || N_AXIS > 3)

static axes_signals_t homing = {0};
static on_homing_rate_set_ptr on_homing_rate_set;
static limits_get_state_ptr limits_get_state;
static on_homing_completed_ptr on_homing_completed;
static limits_enable_ptr limits_enable;

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    if(homing_cycle.mask)
        on = false;

    limits_enable(on, homing_cycle);
}

static limit_signals_t limitsGetState (void)
{
    limit_signals_t state;

    state = limits_get_state();

#if X_AUTO_SQUARE
    if(homing.x && state.min.z) {
        state.min2.x = state.min.z;
        state.min.z = Off;
    } else
        state.min2.x = Off;
#endif

#if Y_AUTO_SQUARE
    if(homing.y && state.min.z) {
        state.min2.y = state.min.z;
        state.min.z = Off;
    } else
        state.min2.y = Off;
#endif

#if Z_AUTO_SQUARE
    if(homing.z && state.min.x) {
        state.min2.z = state.min.x;
        state.min.x = Off;
    } else
        state.min2.z = Off;
#endif

#if N_AXIS == 4
    if(homing.a && state.min.z) {
        state.min.a = state.min.z;
        state.min.z = Off;
    } else
        state.min.a = Off;
#endif

    return state;
}

static void onHomingRateSet (axes_signals_t axes, float rate, homing_mode_t mode)
{
    homing = axes;

    if(on_homing_rate_set)
        on_homing_rate_set(axes, rate, mode);
}

static void onHomingCompleted (bool success)
{
    homing.mask = 0;

    if(on_homing_completed)
        on_homing_completed(success);
}

void board_init (void)
{
    limits_enable = hal.limits.enable;
    hal.limits.enable = limitsEnable;

    limits_get_state = hal.limits.get_state;
    hal.limits.get_state = limitsGetState;

    on_homing_rate_set = grbl.on_homing_rate_set;
    grbl.on_homing_rate_set = onHomingRateSet;

    on_homing_completed = grbl.on_homing_completed;
    grbl.on_homing_completed = onHomingCompleted;
}

#endif // BOARD_BLACKBOX_X32
