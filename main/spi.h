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

#ifndef _GRBL_SPI_H_
#define _GRBL_SPI_H_

#include "driver/sdspi_host.h"
#include "driver/spi_master.h"

void spi_init (void);
bool spi_bus_init (spi_host_device_t *host);
void spi_set_max_speed (void);
uint32_t spi_set_speed (uint32_t prescaler);
uint8_t spi_get_byte (void);
uint8_t spi_put_byte (uint8_t byte);
esp_err_t spi_bus (bool aquire);

#endif
