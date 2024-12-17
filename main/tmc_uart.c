/*
  tmc_uart.c - driver code for ESP32

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

#if TRINAMIC_UART_ENABLE == 1

#include <string.h>

static io_stream_t tmc_uart = {0};

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *dgr)
{
    static TMC_uart_write_datagram_t wdgr = {0};
    volatile uint32_t dly = 50, ms = hal.get_elapsed_ticks();

    if(tmc_uart.write_n == NULL)
        return &wdgr;

#if defined(BOARD_JACKPOT) && defined(M3_UART_CS)

    if(driver.id >= 3) {
        DIGITAL_OUT(M3_UART_CS, driver.id == 3);
#if defined(M4_UART_CS)
        DIGITAL_OUT(M4_UART_CS, driver.id == 4);
#endif
#if defined(M5_UART_CS)
        DIGITAL_OUT(M5_UART_CS, driver.id == 5);
#endif
    }

//    add delay?

#endif // BOARD_JACKPOT

    vTaskSuspendAll();

    tmc_uart.reset_write_buffer();
    tmc_uart.write_n((char *)dgr->data, sizeof(TMC_uart_read_datagram_t));

    while(tmc_uart.get_tx_buffer_count());

    while(--dly);

    tmc_uart.reset_read_buffer();
    tmc_uart.disable_rx(false);

    xTaskResumeAll();

    // Wait for response with 10 ms timeout
    while(tmc_uart.get_rx_buffer_count() < 8) {
        if(hal.get_elapsed_ticks() - ms >= 11)
            break;
    }

    if(tmc_uart.get_rx_buffer_count() >= 8) {

        wdgr.data[0] = tmc_uart.read();
        wdgr.data[1] = tmc_uart.read();
        wdgr.data[2] = tmc_uart.read();
        wdgr.data[3] = tmc_uart.read();
        wdgr.data[4] = tmc_uart.read();
        wdgr.data[5] = tmc_uart.read();
        wdgr.data[6] = tmc_uart.read();
        wdgr.data[7] = tmc_uart.read();

    } else
        wdgr.msg.addr.value = 0xFF;

    tmc_uart.disable_rx(true);

    dly = 5000;
    while(--dly);

    return &wdgr;
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    if(tmc_uart.write_n == NULL)
        return;

#if defined(BOARD_JACKPOT) && defined(M3_UART_CS)

    if(driver.id >= 3) {
        DIGITAL_OUT(M3_UART_CS, driver.id == 3);
#if defined(M4_UART_CS)
        DIGITAL_OUT(M4_UART_CS, driver.id == 4);
#endif
#if defined(M5_UART_CS)
        DIGITAL_OUT(M5_UART_CS, driver.id == 5);
#endif
    }

//    add delay?

#endif // BOARD_JACKPOT

    tmc_uart.reset_read_buffer();
    tmc_uart.write_n((char *)dgr->data, sizeof(TMC_uart_write_datagram_t));

    while(tmc_uart.get_tx_buffer_count());
}

#if defined(BOARD_JACKPOT) && defined(M3_UART_CS)

static void driver_preinit (motor_map_t motor, trinamic_driver_config_t *config)
{
    if(motor.id > 3)
        config->address = 3;
}

#endif // BOARD_JACKPOT

void tmc_uart_init (void)
{
#if defined(BOARD_JACKPOT) && defined(M3_UART_CS)

    static trinamic_driver_if_t driver_if = {
        .on_driver_preinit = driver_preinit
    };

    trinamic_if_init(&driver_if);

#endif // BOARD_JACKPOT

    const io_stream_t *stream;

    if((stream = stream_open_instance(TRINAMIC_STREAM, 230400, NULL, "Trinamic UART")) == NULL)
        stream = stream_null_init(230400);

    memcpy(&tmc_uart, stream, sizeof(io_stream_t));
    tmc_uart.disable_rx(true);
    tmc_uart.set_enqueue_rt_handler(stream_buffer_all);

// if no stream output -> POS failure?

}

#elif TRINAMIC_UART_ENABLE

#error "Soft UART Trinamic driver not available!"

#endif
