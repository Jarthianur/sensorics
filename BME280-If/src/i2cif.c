#include "i2cif.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

int i2c_init_dev(const char* dev, uint8_t addr)
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
    uint8_t args[2] = { reg, data };
    if (i2c_smbus_write_block_data(fd, 0, 2, args) < 0)
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
    uint8_t args[bytes + 1] = { 0 };
    args[0] = reg;
    for (size_t i = 0; i < bytes; ++i)
    {
        args[i + 1] = data[i];
    }
    if (i2c_smbus_write_block_data(fd, 0, bytes + 1, args) < 0)
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
    if ((*data = (uint8_t) (i2c_smbus_read_byte(fd) & 0xFF)) < 0)
    {
        return ERROR;
    }
    return SUCCESS;
}

int32_t i2c_read_block(int fd, uint8_t reg, size_t bytes, uint8_t* data)
{
    if (i2c_smbus_write_byte(fd, reg) < 0)
    {
        return ERROR;
    }
    if (i2c_smbus_read_block_data(fd, 0, data) < 0)
    {
        return ERROR;
    }
    return SUCCESS;
}

