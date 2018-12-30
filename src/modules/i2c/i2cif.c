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

#include "i2c/i2cif.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

int I2C_init_dev(const char* dev, u8_t addr)
{
    int fd;
    if ((fd = open(dev, O_RDWR)) < 0)
    {
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, addr) < 0)
    {
        return -1;
    }
    return fd;
}

void I2C_close_dev(int fd)
{
    close(fd);
}

s32_t I2C_write_reg(int fd, u8_t reg, u8_t data)
{
    if (i2c_smbus_write_byte_data(fd, reg, data) < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

s32_t I2C_write_block(int fd, u8_t reg, size_t bytes, u8_t* data)
{
    if (i2c_smbus_write_i2c_block_data(fd, reg, bytes, data) < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

s32_t I2C_read_reg(int fd, u8_t reg, u8_t* data)
{
    if (i2c_smbus_write_byte(fd, reg) < 0)
    {
        return -1;
    }
    s32_t v = i2c_smbus_read_byte(fd);
    if (v < 0)
    {
        return -1;
    }
    *data = v & 0x0FF;
    return 0;
}

s32_t I2C_read_block(int fd, u8_t reg, size_t bytes, u8_t* data)
{
    if (i2c_smbus_read_i2c_block_data(fd, reg, bytes, data) <= 0)
    {
        return -1;
    }
    return 0;
}