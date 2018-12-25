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

int32_t i2c_init_dev(const char* dev, uint8_t addr)
{
    int fd;
    if ((fd = open(dev, O_RDWR)) < 0)
    {
        return ERROR;
    }
    if (ioctl(fd, I2C_SLAVE, addr) < 0)
    {
        return ERROR;
    }
    return fd;
}

void i2c_close_dev(int fd)
{
    close(fd);
}

int32_t i2c_write_reg(int fd, uint8_t reg, uint8_t data)
{
    if (i2c_smbus_write_byte_data(fd, reg, data) < 0)
    {
        return ERROR;
    }
    else
    {
        return SUCCESS;
    }
}

int32_t i2c_write_block(int fd, uint8_t reg, size_t bytes, uint8_t* data)
{
    if (i2c_smbus_write_i2c_block_data(fd, reg, bytes, data) < 0)
    {
        return ERROR;
    }
    else
    {
        return SUCCESS;
    }
}

int32_t i2c_read_reg(int fd, uint8_t reg, uint8_t* data)
{
    if (i2c_smbus_write_byte(fd, reg) < 0)
    {
        return ERROR;
    }
    if ((*data = i2c_smbus_read_byte(fd)) < 0)
    {
        return ERROR;
    }
    return SUCCESS;
}

int32_t i2c_read_block(int fd, uint8_t reg, size_t bytes, uint8_t* data)
{
    if (i2c_smbus_read_i2c_block_data(fd, reg, bytes, data) <= 0)
    {
        return ERROR;
    }
    return SUCCESS;
}
