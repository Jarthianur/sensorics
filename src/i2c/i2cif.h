#ifndef SRC_I2C_I2CIF_H_
#define SRC_I2C_I2CIF_H_

#define I2C_API

#define ERROR   (-1)
#define SUCCESS (0)

#include <stdint.h>
#include <stddef.h>

/**
 * Open file-descriptor to device dev at addr and enable io control over i2c bus.
 * @return ERROR on any error, else file-descriptor
 */
int i2c_init_dev(const char* dev, uint8_t addr);
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

#endif /* SRC_I2C_I2CIF_H_ */
