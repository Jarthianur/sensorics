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

#include "util/types.h"

/**
 * Open file-descriptor to device dev at addr and enable io control over i2c bus.
 */
int I2C_init_dev(const char* dev, u8_t addr);

/**
 * Close device fd.
 */
void I2C_close_dev(int fd);

/**
 * Write 1 Byte of data to device fd at register reg.
 */
s32_t I2C_write_reg(int fd, u8_t reg, u8_t data);

/**
 * Write block of data to device fd at register reg.
 */
s32_t I2C_write_block(int fd, u8_t reg, size_t bytes, u8_t* data);

/**
 * Read 1 Byte of data from device fd at register reg.
 */
s32_t I2C_read_reg(int fd, u8_t reg, u8_t* data);

/**
 * Read block of data to device fd at register reg.
 */
s32_t I2C_read_block(int fd, u8_t reg, size_t bytes, u8_t* data);
