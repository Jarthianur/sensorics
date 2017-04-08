#include "bme280.h"
#include "i2cif.h"
#include <stddef.h>

#define NULL_CHECK if (check_null() == (U8_ERROR)) return (U8_ERROR);

static bme280* p_bme280 = NULL;

uint8_t bme280_init(const char* dev, bme280* inst)
{
    p_bme280 = inst;
    NULL_CHECK
    if ((p_bme280->fd = i2c_init_dev(dev, BME280_ADDRESS)) == ERROR)
    {
        return U8_ERROR;
    }
    return U8_SUCCESS;
}

uint8_t bme280_deinit()
{
    NULL_CHECK
    i2c_close_dev(p_bme280->fd);
    return U8_SUCCESS;
}

uint8_t bme280_read_burst_tph()
{
    NULL_CHECK
    uint8_t* buff = p_bme280->buffer;
    bme280_uncomp_meas* meas = p_bme280->p_uncomp_meas;
    if (i2c_read_block(p_bme280->fd, BME280_REG_MEAS, 8, buff) == ERROR)
    {
        return U8_ERROR;
    }
    meas->pmsb = buff[0];
    meas->plsb = buff[1];
    meas->pxsb = buff[2];
    meas->tmsb = buff[3];
    meas->tlsb = buff[4];
    meas->txsb = buff[5];
    meas->hmsb = buff[6];
    meas->hlsb = buff[7];
    unpack_press();
    unpack_temp();
    unpack_humid();
    return U8_SUCCESS;
}

uint8_t bme280_read_temp()
{
    NULL_CHECK
    uint8_t* buff = p_bme280->buffer;
    bme280_uncomp_meas* meas = p_bme280->p_uncomp_meas;
    if (i2c_read_block(p_bme280->fd, BME280_REG_TEMP, 3, buff) == ERROR)
    {
        return U8_ERROR;
    }
    meas->tmsb = buff[0];
    meas->tlsb = buff[1];
    meas->txsb = buff[2];
    return unpack_temp();
}

uint8_t bme280_read_press()
{
    NULL_CHECK
    uint8_t* buff = p_bme280->buffer;
    bme280_uncomp_meas* meas = p_bme280->p_uncomp_meas;
    if (i2c_read_block(p_bme280->fd, BME280_REG_PRESS, 3, buff) == ERROR)
    {
        return U8_ERROR;
    }
    meas->pmsb = buff[0];
    meas->plsb = buff[1];
    meas->pxsb = buff[2];
    return unpack_press();
}

uint8_t bme280_read_humid()
{
    NULL_CHECK
    uint8_t* buff = p_bme280->buffer;
    bme280_uncomp_meas* meas = p_bme280->p_uncomp_meas;
    if (i2c_read_block(p_bme280->fd, BME280_REG_HUMID, 2, buff) == ERROR)
    {
        return U8_ERROR;
    }
    meas->hmsb = buff[0];
    meas->hlsb = buff[1];
    return unpack_humid();
}

uint8_t bme280_powermode(uint8_t mode)
{
}

uint8_t bme280_powermode()
{
    NULL_CHECK
    return p_bme280->power_mode;
}

uint8_t bme280_temp_oversample(uint8_t rate)
{
}

uint8_t bme280_temp_oversample()
{
    NULL_CHECK
    return p_bme280->ovrsmpl_temp;
}

uint8_t bme280_press_oversample(uint8_t rate)
{
}

uint8_t bme280_press_oversample()
{
    NULL_CHECK
    return p_bme280->ovrsmpl_press;
}

uint8_t bme280_humid_oversample(uint8_t rate)
{
}

uint8_t bme280_humid_oversample()
{
    NULL_CHECK
    return p_bme280->ovrsmpl_humid;
}

uint8_t bme280_standby_durn(uint8_t ms)
{
}

uint8_t bme280_standby_durn()
{
    NULL_CHECK
    return p_bme280->standby_durn;
}

uint8_t bme280_filter(uint8_t coef)
{
}

uint8_t bme280_filter()
{
    NULL_CHECK
    return p_bme280->filter;
}

uint8_t bme280_compensate_temp()
{
}

uint8_t bme280_compensate_press()
{
}

uint8_t bme280_compensate_humid()
{
}

double bme280_temp()
{
    NULL_CHECK
    bme280_compensate_temp();
    return (double) p_bme280->comp_temp;
}

double bme280_press()
{
    NULL_CHECK
    bme280_compensate_press();
    return (double) p_bme280->comp_press;
}

double bme280_humid()
{
    NULL_CHECK
    bme280_compensate_humid();
    return (double) p_bme280->comp_humid;
}

uint8_t bme280_soft_rst()
{
    NULL_CHECK
    if (i2c_write_reg(p_bme280->fd, BME280_REG_SOFTRST, BME280_RST) == ERROR)
    {
        return U8_ERROR;
    }
    else
    {
        return U8_SUCCESS;
    }
}

uint8_t unpack_press()
{
    NULL_CHECK
    bme280_uncomp_meas* meas = p_bme280->p_uncomp_meas;
    meas->pressure = (int32_t) ((((uint32_t) (meas->pmsb)) << 12)
            | (((uint32_t) (meas->plsb)) << 4) | ((uint32_t) meas->pxsb >> 4));
    return U8_SUCCESS;
}

uint8_t unpack_temp()
{
    NULL_CHECK
    bme280_uncomp_meas* meas = p_bme280->p_uncomp_meas;
    meas->temperature = (int32_t) ((((uint32_t) (meas->tmsb)) << 12)
            | (((uint32_t) (meas->tlsb)) << 4) | ((uint32_t) meas->txsb >> 4));
    return U8_SUCCESS;
}

uint8_t unpack_humid()
{
    NULL_CHECK
    bme280_uncomp_meas* meas = p_bme280->p_uncomp_meas;
    meas->humidity = (int32_t) ((((uint32_t) (meas->hmsb)) << 8)
            | ((uint32_t) (meas->hlsb)));
    return U8_SUCCESS;
}

uint8_t check_null()
{
    if (p_bme280 == NULL)
    {
        return U8_ERROR;
    }
    if (p_bme280->p_calib == NULL)
    {
        return U8_ERROR;
    }
    if (p_bme280->p_uncomp_meas == NULL)
    {
        return U8_ERROR;
    }
    return U8_SUCCESS;
}

