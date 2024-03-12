/*
  ioports_analog.c - driver code for ESP32

  Part of grblHAL

  Copyright (c) 2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public Licens
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#include "grbl/ioports.h"

#ifdef AUXOUTPUT0_PWM_PIN
#define PWM_OUT0 1
#else
#define PWM_OUT0 0
#endif

#ifdef AUXOUTPUT1_PWM_PIN
#define PWM_OUT1 1
#else
#define PWM_OUT1 0
#endif

#define AUX_ANALOG_OUT (PWM_OUT0 + PWM_OUT1)

#if defined(AUXINPUT0_ANALOG_PIN) ||  defined(AUXINPUT1_ANALOG_PIN)
#define AUX_ANALOG_IN 1
#else
#define AUX_ANALOG_IN 0
#endif

//#if AUX_ANALOG_IN || AUX_ANALOG_OUT

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;

static set_pin_description_ptr set_pin_description_digital;
static get_pin_info_ptr get_pin_info_digital;
static claim_port_ptr claim_digital;
//static swap_pins_ptr swap_pins_digital;
#if AUX_ANALOG_IN
static wait_on_input_ptr wait_on_input_digital;
#endif

//#endif

#if AUX_ANALOG_IN

#if CONFIG_IDF_TARGET_ESP32

static const adc_map_t adc_map[] = {
    { ADC1_CHANNEL_0, GPIO_NUM_36 },
    { ADC1_CHANNEL_1, GPIO_NUM_37 },
    { ADC1_CHANNEL_2, GPIO_NUM_38 },
    { ADC1_CHANNEL_3, GPIO_NUM_39 },
    { ADC1_CHANNEL_4, GPIO_NUM_32 },
    { ADC1_CHANNEL_5, GPIO_NUM_33 },
    { ADC1_CHANNEL_6, GPIO_NUM_34 },
    { ADC1_CHANNEL_7, GPIO_NUM_35 }
};

#elif CONFIG_IDF_TARGET_ESP32S3

static const adc_map_t adc_map[] = {
    { ADC1_CHANNEL_0, GPIO_NUM_1 },
    { ADC1_CHANNEL_1, GPIO_NUM_2 },
    { ADC1_CHANNEL_2, GPIO_NUM_3 },
    { ADC1_CHANNEL_3, GPIO_NUM_4 },
    { ADC1_CHANNEL_4, GPIO_NUM_5 },
    { ADC1_CHANNEL_5, GPIO_NUM_6 },
    { ADC1_CHANNEL_6, GPIO_NUM_7 },
    { ADC1_CHANNEL_7, GPIO_NUM_8 },
    { ADC1_CHANNEL_8, GPIO_NUM_9 },
    { ADC1_CHANNEL_9, GPIO_NUM_10 },
};

#endif
#endif // AUX_ANALOG_IN

#ifdef MCP3221_ENABLE

#include "MCP3221.h"

static xbar_t analog_in;
static enumerate_pins_ptr on_enumerate_pins;

static void enumerate_pins (bool low_level, pin_info_ptr pin_info, void *data)
{
    on_enumerate_pins(low_level, pin_info, data);

    pin_info(&analog_in, data);
}

#endif // MCP3221_ENABLE

#if AUX_ANALOG_OUT

typedef struct {
    uint32_t max_value;
    ledc_channel_config_t ch_config;
    ioports_pwm_t data;
    float value;
    void (*set_value)(uint_fast8_t ch, float value);
} pwm_out_t;

static pwm_out_t pwm_out[AUX_ANALOG_OUT] = {0};

static float pwm_get_value (struct xbar *output)
{
    int_fast8_t ch = output->function - Output_Analog_Aux0;

    return ch >= 0 && ch < analog.out.n_ports ? pwm_out[ch].value : -1.0f;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {

        port = ioports_map(analog.out, port);

        uint_fast8_t ch = aux_out_analog[port].id - Output_Analog_Aux0;

        pwm_out[ch].set_value(ch, value);
    }

    return port < analog.out.n_ports;
}

static void pwm_set_value (uint_fast8_t ch, float value)
{
    uint_fast16_t pwm_value = ioports_compute_pwm_value(&pwm_out[ch].data, value);

    pwm_out[ch].value = value;

    if(pwm_value == pwm_out[ch].data.off_value) {
        if(pwm_out[ch].data.always_on) {
            ledc_set_duty(pwm_out[ch].ch_config.speed_mode, pwm_out[ch].ch_config.channel, pwm_out[ch].data.off_value);
            ledc_update_duty(pwm_out[ch].ch_config.speed_mode, pwm_out[ch].ch_config.channel);
        } else
            ledc_stop(pwm_out[ch].ch_config.speed_mode, pwm_out[ch].ch_config.channel, 0);
    } else {
        ledc_set_duty(pwm_out[ch].ch_config.speed_mode, pwm_out[ch].ch_config.channel, pwm_value);
        ledc_update_duty(pwm_out[ch].ch_config.speed_mode, pwm_out[ch].ch_config.channel);
    }
}

#endif

#ifdef AUXOUTPUT0_PWM_PIN

static bool init_pwm0 (xbar_t *pin, pwm_config_t *config, bool persistent)
{
    static bool init_ok = false;

    bool ok;
    uint_fast8_t ch = pin->function - Output_Analog_Aux0;
    ioports_pwm_t *pwm_data = &pwm_out[ch].data;
    ledc_channel_config_t *ch_config = &pwm_out[ch].ch_config;

    static ledc_timer_config_t pwm_timer = {
#if CONFIG_IDF_TARGET_ESP32S3
        .speed_mode = LEDC_LOW_SPEED_MODE,
#else
        .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 5000
    };

    if(!init_ok) {

        init_ok = true;

        ch_config->gpio_num = AUXOUTPUT0_PWM_PIN,
#if CONFIG_IDF_TARGET_ESP32S3
        ch_config->speed_mode = LEDC_SPEED_MODE_MAX;
#else
        ch_config->speed_mode = LEDC_HIGH_SPEED_MODE;
#endif
        ch_config->channel = LEDC_CHANNEL_1;
        ch_config->intr_type = LEDC_INTR_DISABLE;
        ch_config->timer_sel = pwm_timer.timer_num;
        ch_config->speed_mode = pwm_timer.speed_mode;

        ledc_timer_config(&pwm_timer);
        ledc_channel_config(ch_config);

        pwm_out[ch].set_value = pwm_set_value;
    }

    if(pwm_timer.freq_hz != (uint32_t)config->freq_hz) {
        pwm_timer.freq_hz = (uint32_t)config->freq_hz;
        if(pwm_timer.freq_hz <= 100) {
#if SOC_LEDC_TIMER_BIT_WIDE_NUM > 14
            if(pwm_timer.duty_resolution != LEDC_TIMER_16_BIT) {
                pwm_timer.duty_resolution = LEDC_TIMER_16_BIT;
                ledc_timer_config(&pwm_timer);
            }
#else
            if(pwm_timer.duty_resolution != LEDC_TIMER_14_BIT) {
                pwm_timer.duty_resolution = LEDC_TIMER_14_BIT;
                ledc_timer_config(&pwm_timer);
            }
#endif
        } else if(pwm_timer.duty_resolution != LEDC_TIMER_10_BIT) {
            pwm_timer.duty_resolution = LEDC_TIMER_10_BIT;
            ledc_timer_config(&pwm_timer);
        }
    }

    if((ok = ledc_set_freq(pwm_timer.speed_mode, pwm_timer.timer_num, pwm_timer.freq_hz) == ESP_OK)) {

        pwm_out[ch].max_value = (1UL << pwm_timer.duty_resolution) - 1;
        ioports_precompute_pwm_values(config, pwm_data, pwm_out[ch].max_value * config->freq_hz);

        aux_out_analog[ch].mode.pwm = !config->servo_mode;
        aux_out_analog[ch].mode.servo_pwm = config->servo_mode;

        pwm_set_value(ch, config->min);
    }

    return ok;
}

#endif // AUXOUTPUT0_PWM_PIN

#ifdef AUXOUTPUT1_PWM_PIN

static bool init_pwm1 (xbar_t *pin, pwm_config_t *config, bool persistent)
{
    static bool init_ok = false;

    bool ok;
    uint_fast8_t ch = pin->function - Output_Analog_Aux0;
    ioports_pwm_t *pwm_data = &pwm_out[ch].data;
    ledc_channel_config_t *ch_config = &pwm_out[ch].ch_config;

    static ledc_timer_config_t pwm_timer = {
#if CONFIG_IDF_TARGET_ESP32S3
        .speed_mode = LEDC_LOW_SPEED_MODE,
#else
        .speed_mode = LEDC_HIGH_SPEED_MODE,
#endif
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_2,
        .freq_hz = 5000
    };

    if(!init_ok) {

        init_ok = true;

        ch_config->gpio_num = AUXOUTPUT1_PWM_PIN,
#if CONFIG_IDF_TARGET_ESP32S3
        ch_config->speed_mode = LEDC_SPEED_MODE_MAX;
#else
        ch_config->speed_mode = LEDC_HIGH_SPEED_MODE;
#endif
        ch_config->channel = LEDC_CHANNEL_2;
        ch_config->intr_type = LEDC_INTR_DISABLE;
        ch_config->timer_sel = pwm_timer.timer_num;
        ch_config->speed_mode = pwm_timer.speed_mode;

        ledc_timer_config(&pwm_timer);
        ledc_channel_config(ch_config);

        pwm_out[ch].set_value = pwm_set_value;
    }

    if(pwm_timer.freq_hz != (uint32_t)config->freq_hz) {
        pwm_timer.freq_hz = (uint32_t)config->freq_hz;
        if(pwm_timer.freq_hz <= 100) {
#if SOC_LEDC_TIMER_BIT_WIDE_NUM > 14
            if(pwm_timer.duty_resolution != LEDC_TIMER_16_BIT) {
                pwm_timer.duty_resolution = LEDC_TIMER_16_BIT;
                ledc_timer_config(&pwm_timer);
            }
#else
            if(pwm_timer.duty_resolution != LEDC_TIMER_14_BIT) {
                pwm_timer.duty_resolution = LEDC_TIMER_14_BIT;
                ledc_timer_config(&pwm_timer);
            }
#endif
        } else if(pwm_timer.duty_resolution != LEDC_TIMER_10_BIT) {
            pwm_timer.duty_resolution = LEDC_TIMER_10_BIT;
            ledc_timer_config(&pwm_timer);
        }
    }

    if((ok = ledc_set_freq(pwm_timer.speed_mode, pwm_timer.timer_num, pwm_timer.freq_hz) == ESP_OK)) {

        pwm_out[ch].max_value = (1UL << pwm_timer.duty_resolution) - 1;
        ioports_precompute_pwm_values(config, pwm_data, pwm_out[ch].max_value * config->freq_hz);

        aux_out_analog[ch].mode.pwm = !config->servo_mode;
        aux_out_analog[ch].mode.servo_pwm = config->servo_mode;

        pwm_set_value(ch, config->min);
    }

    return ok;
}

#endif // AUXOUTPUT1_PWM_PIN

#if AUX_ANALOG_IN

static int32_t wait_on_input_dummy (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return -1;
}

static int32_t wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(type == Port_Digital)
        return wait_on_input_digital(type, port, wait_mode, timeout);

    port = ioports_map(analog.in, port);

#ifdef MCP3221_ENABLE
    if(port == analog_in.pin)
        value = (int32_t)MCP3221_read();
    else
#endif

    if(port < analog.in.n_ports && aux_in_analog[port].adc)
        value = adc1_get_raw(aux_in_analog[port].adc->ch);

    return value;
}

#endif

static xbar_t *get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    if(type == Port_Digital)
        return get_pin_info_digital ? get_pin_info_digital(type, dir, port) : NULL;

    else switch(dir) {

        case Port_Input:
            if(port < analog.in.n_ports) {
                port = ioports_map(analog.in, port);
#ifdef MCP3221_ENABLE
                if(port == analog_in.pin)
                    info = &analog_in;
                else
#endif
                if(aux_in_analog[port].cap.analog) {
                    pin.id = port;
                    pin.mode = aux_in_analog[pin.id].mode;
                    pin.cap = aux_in_analog[pin.id].cap;
                    pin.cap.claimable = !pin.mode.claimed;
                    pin.function = aux_in_analog[pin.id].id;
                    pin.group = aux_in_analog[pin.id].group;
                    pin.pin = aux_in_analog[pin.id].pin;
                    pin.description = aux_in_analog[pin.id].description;
                    info = &pin;
                }
            }
            break;

        case Port_Output:
#if AUX_ANALOG_OUT
            memset(&pin, 0, sizeof(xbar_t));

            if(port < analog.out.n_ports) {
                pin.id = ioports_map(analog.out, port);
                pin.mode = aux_out_analog[pin.id].mode;
                pin.mode.pwm = !pin.mode.servo_pwm; //?? for easy filtering
                XBAR_SET_CAP(pin.cap, pin.mode);
                pin.function = aux_out_analog[pin.id].id;
                pin.group = aux_out_analog[pin.id].group;
                pin.pin = aux_out_analog[pin.id].pin;
                pin.description = aux_out_analog[pin.id].description;
                pin.get_value = pwm_get_value;
    #ifdef AUXOUTPUT0_PWM_PIN
                if(aux_out_analog[pin.id].pin == AUXOUTPUT0_PWM_PIN)
                    pin.config = init_pwm0;
    #endif
    #ifdef AUXOUTPUT1_PWM_PIN
                if(aux_out_analog[pin.id].pin == AUXOUTPUT1_PWM_PIN)
                    pin.config = init_pwm1;
    #endif
                info = &pin;
            }
#endif // AUX_ANALOG_OUT
            break;
    }

    return info;
}

static void set_pin_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *description)
{
    if(type == Port_Analog) {
        if(dir == Port_Input && port < analog.in.n_ports) {
            port = ioports_map(analog.in, port);
#ifdef MCP3221_ENABLE
            if(port == analog_in.pin)
                analog_in.description = description;
            else
#endif
            aux_in_analog[port].description = description;
        } else if(port < analog.out.n_ports)
            aux_out_analog[ioports_map(analog.out, port)].description = description;
    } else if(set_pin_description_digital)
        set_pin_description_digital(type, dir, port, description);
}

static bool claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;

    if(type == Port_Digital)
        return claim_digital ? claim_digital(type, dir, port, description) : false;

    else switch(dir) {

        case Port_Input:

            if((ok = analog.in.map && *port < analog.in.n_ports && aux_in_analog[*port].cap.analog && !(
#ifdef MCP3221_ENABLE
                    *port == analog_in.pin ? analog_in.mode.claimed :
#endif
                    aux_in_analog[*port].mode.claimed))) {

                uint8_t i;

                hal.port.num_analog_in--;

                for(i = ioports_map_reverse(&analog.in, *port); i < hal.port.num_analog_in; i++) {
                    analog.in.map[i] = analog.in.map[i + 1];
#ifdef MCP3221_ENABLE
                    if(analog_in.pin == analog.in.map[i])
                        analog_in.description = iports_get_pnum(analog, i);
                    else
#endif
                    aux_in_analog[analog.in.map[i]].description = iports_get_pnum(analog, i);
                }

#ifdef MCP3221_ENABLE
                if(*port == analog_in.pin) {
                    analog_in.mode.claimed = On;
                    analog_in.description = description;
                } else
#endif
                {
                    aux_in_analog[*port].mode.claimed = On;
                    aux_in_analog[*port].description = description;
                }
                analog.in.map[hal.port.num_analog_in] = *port;
                *port = hal.port.num_analog_in;
            }
            break;

        case Port_Output:
#if AUX_ANALOG_OUT
            if((ok = analog.out.map && *port < analog.out.n_ports && !aux_out_analog[*port].mode.claimed)) {

                uint8_t i;

                hal.port.num_analog_out--;

                for(i = ioports_map_reverse(&analog.out, *port); i < hal.port.num_analog_out; i++) {
                    analog.out.map[i] = analog.out.map[i + 1];
                    aux_out_analog[analog.out.map[i]].description = iports_get_pnum(analog, i);
                }

                aux_out_analog[*port].mode.claimed = On;
                aux_out_analog[*port].description = description;

                analog.out.map[hal.port.num_analog_out] = *port;
                *port = hal.port.num_analog_out;
            }
#endif
            break;
    }

    return ok;
}

void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    aux_in_analog = aux_inputs->pins.inputs;
    aux_out_analog = aux_outputs->pins.outputs;

    set_pin_description_digital = hal.port.set_pin_description;
    hal.port.set_pin_description = set_pin_description;

#ifdef MCP3221_ENABLE

    pin_group_pins_t aux_in = {
        .n_pins = 1
    };

    analog_in.function = Input_Analog_Aux0 + aux_inputs->n_pins;
    analog_in.group = PinGroup_AuxInput;
    analog_in.pin = aux_inputs->n_pins;
    analog_in.port = "MCP3221:";

    if(MCP3221_init()) {
        analog_in.mode.analog = On;
        if(aux_inputs)
            aux_inputs->n_pins++;
        else
            aux_inputs = &aux_in;
    } else
        analog_in.description = "No power";

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = enumerate_pins;

#endif // MCP3221_ENABLE

    if(ioports_add(&analog, Port_Analog, aux_inputs->n_pins, aux_outputs->n_pins)) {

#if AUX_ANALOG_IN

        uint_fast8_t p_pins = aux_inputs->n_pins;

        if(p_pins) {

            bool ok;
            uint_fast8_t i, j;

            for(i = 0; i < p_pins; i++) {

                ok = false;
                for(j = 0; i < sizeof(adc_map) / sizeof(adc_map_t); j++) {

                    if((ok = adc_map[i].pin == aux_in_analog[i].pin)) {
                        adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
                        adc1_config_channel_atten(adc_map[i].ch, ADC_ATTEN_DB_11);
                        aux_in_analog[i].adc = &adc_map[i];
                        aux_in_analog[i].cap.analog = On;
                        break;
                    }
                }
                if(!ok)
                    analog.in.n_ports--; // TODO: claim port?
            }
        }


        if(analog.in.n_ports) {
            if((wait_on_input_digital = hal.port.wait_on_input) == NULL)
                wait_on_input_digital = wait_on_input_dummy;
            hal.port.wait_on_input = wait_on_input;
        }

#endif

#if AUX_ANALOG_OUT

        if(analog.out.n_ports) {

            xbar_t *pin;
            uint_fast8_t i;
            pwm_config_t config = {
                .freq_hz = 5000.0f,
                .min = 0.0f,
                .max = 100.0f,
                .off_value = 0.0f,
                .min_value = 0.0f,
                .max_value = 100.0f,
                .invert = Off
            };

            hal.port.analog_out = analog_out;

            for(i = 0; i < analog.out.n_ports; i++) {
                if((pin = get_pin_info(Port_Analog, Port_Output, i)))
                    pin->config(pin, &config, false);
            }
        }

#endif // AUX_ANALOG_OUT

        claim_digital = hal.port.claim;
        hal.port.claim = claim;

        get_pin_info_digital = hal.port.get_pin_info;
        hal.port.get_pin_info = get_pin_info;
//        swap_pins = hal.port.swap_pins;
//        hal.port.swap_pins = swap_pins;

    } else
        hal.port.set_pin_description = set_pin_description_digital;
}
