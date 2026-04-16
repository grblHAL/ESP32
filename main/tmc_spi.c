/*
  tmc_spi.c - driver code for ESP32

  Part of grblHAL

  Copyright (c) 2020-2026 Terje Io

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

#if TRINAMIC_SPI_ENABLE

#include <math.h>
#include <string.h>

#include "spi.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

#include "driver/spi_master.h"

static volatile uint32_t dly = 100;

static inline void delay (uint32_t delay)
{
    dly = delay;
    while(--dly);
}

static spi_slave_t dev = {
    .f_clock = SPI_MASTER_FREQ_10M
};

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_CS_SINGLE

static uint_fast8_t n_motors;
static TMC_spi_datagram_t datagram[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    static TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;

    datagram[driver.seq].addr.value = reg->addr.value;
    datagram[driver.seq].addr.write = 0;

    spi_select(&dev);

    do {
        spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
    } while(idx);

    delay(100);
    spi_deselect(&dev);
    delay(50);
    spi_select(&dev);

    idx = n_motors;
    do {
        res = spi_put_byte(datagram[--idx].addr.value);

        if(idx == driver.seq) {
            status = res;
            reg->payload.data[3] = spi_get_byte();
            reg->payload.data[2] = spi_get_byte();
            reg->payload.data[1] = spi_get_byte();
            reg->payload.data[0] = spi_get_byte();
        } else {
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
        }
    } while(idx);

    delay(100);
    spi_deselect(&dev);
    delay(50);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;

    memcpy(&datagram[driver.seq], reg, sizeof(TMC_spi_datagram_t));
    datagram[driver.seq].addr.write = 1;

    spi_select(&dev);

    do {
        res = spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(datagram[idx].payload.data[3]);
        spi_put_byte(datagram[idx].payload.data[2]);
        spi_put_byte(datagram[idx].payload.data[1]);
        spi_put_byte(datagram[idx].payload.data[0]);

        if(idx == driver.seq) {
            status = res;
            datagram[idx].addr.idx = 0; // TMC_SPI_STATUS_REG;
            datagram[idx].addr.write = 0;
        }
    } while(idx);

    delay(100);
    spi_deselect(&dev);
    delay(50);

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if(gpio->function == Output_MotorChipSelect)
        dev.cs_pin = gpio->pin + (gpio->port ? I2S_OUT_PIN_BASE : 0);
}

static void if_init (uint8_t motors, axes_signals_t axisflags)
{
    n_motors = motors;

    hal.enumerate_pins(true, add_cs_pin, NULL);
}

void tmc_spi_init (void)
{
    trinamic_driver_if_t driver = {
        .on_drivers_init = if_init
    };

    spi_start(&dev);

    uint_fast8_t idx = TMC_N_MOTORS_MAX;
    do {
        datagram[--idx].addr.idx = 0; //TMC_SPI_STATUS_REG;
    } while(idx);

    trinamic_if_init(&driver);
}

#else

static struct {
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    dev.cs_pin = cs[driver.id].pin;
    spi_select(&dev);
    delay(100);

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    spi_deselect(&dev);
    delay(100);
    spi_select(&dev);

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    spi_deselect(&dev);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    dev.cs_pin = cs[driver.id].pin;
    spi_select(&dev);
    delay(100);

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    spi_deselect(&dev);

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if(gpio->group == PinGroup_MotorChipSelect) {

        if(gpio->port)
            gpio->pin += I2S_OUT_PIN_BASE;

        switch (gpio->function) {

            case Output_MotorChipSelectX:
                cs[X_AXIS].pin = gpio->pin;
                break;

            case Output_MotorChipSelectY:
                cs[Y_AXIS].pin = gpio->pin;
                break;

            case Output_MotorChipSelectZ:
                cs[Z_AXIS].pin = gpio->pin;
                break;

            case Output_MotorChipSelectM3:
                cs[3].pin = gpio->pin;
                break;

            case Output_MotorChipSelectM4:
                cs[4].pin = gpio->pin;
                break;

            case Output_MotorChipSelectM5:
                cs[5].pin = gpio->pin;
                break;

            default:
                break;
        }
    }
}

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    hal.enumerate_pins(false, add_cs_pin, NULL);
}

void tmc_spi_init (void)
{
    static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

    spi_start(&dev);
    trinamic_if_init(&driver_if);
}

#endif

#endif // TRINAMIC_SPI_ENABLE
