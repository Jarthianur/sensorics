/*
 Copyright_License {

 Copyright (C) 2017 Julian P. Becht
 Author: Julian P. Becht

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License version 3
 as published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 }
 */

#pragma once

#define I2C_API

#define ERROR   (-1)
#define SUCCESS (0)

#include <stdint.h>
#include <stddef.h>

/**
 * Open file-descriptor to device dev at addr and enable io control over i2c bus.
 * @return ERROR on any error, else file-descriptor
 */
int32_t i2c_init_dev(const char* dev, uint8_t addr);
/**
 * Close device fd.
 */
void i2c_close_dev(int fd);
/**
 * Write 1 Byte of data to device fd at register reg.
 * @return ERROR on any error, else SUCCESS
 */
int32_t i2c_write_reg(int fd, uint8_t reg, uint8_t data);
/**
 * Write block of data to device fd at register reg.
 * @return ERROR on any error, else SUCCESS
 */
int32_t i2c_write_block(int fd, uint8_t reg, size_t bytes, uint8_t* data);
/**
 * Read 1 Byte of data from device fd at register reg.
 * @return ERROR on any error, else SUCCESS
 */
int32_t i2c_read_reg(int fd, uint8_t reg, uint8_t* data);
/**
 * Read block of data to device fd at register reg.
 * @return ERROR on any error, else SUCCESS
 */
int32_t i2c_read_block(int fd, uint8_t reg, size_t bytes, uint8_t* data);
