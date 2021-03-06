/*
 Author: Julian P. Becht

 This program is free software; you can redistribute it and/or
 modify it.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 */

#define _POSIX_C_SOURCE 200809L
#include "bme280.h"
#include "../i2c/i2cif.h"
#include <stddef.h>
#include <unistd.h>
#include <time.h>

#define NULL_CHECK                  \
    if (check_null() == (U8_ERROR)) \
        return (U8_ERROR);

/**
 * Unpack pressure from uncomp struct.
 */
uint8_t unpack_press();
/**
 * Unpack temperature from uncomp struct.
 */
uint8_t unpack_temp();
/**
 * Unpack humidity from uncomp struct.
 */
uint8_t unpack_humid();
/**
 * Check whether all bme280 related pointers are valid.
 */
uint8_t check_null();
/**
 * Bit slicing
 */
uint8_t set_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos, uint8_t val);
uint8_t get_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos);

/**
 * THE bme280 ptr.
 */
static bme280 *p_bme280 = NULL;

uint8_t bme280_init(const char *dev, bme280 *inst)
{
    p_bme280 = inst;
    NULL_CHECK
    if ((p_bme280->fd = i2c_init_dev(dev, BME280_ADDRESS)) == ERROR)
    {
        return U8_ERROR;
    }
    return bme280_read_calib();
}

uint8_t bme280_deinit()
{
    NULL_CHECK
    i2c_close_dev(p_bme280->fd);
    return U8_SUCCESS;
}

uint8_t bme280_read_calib()
{
    NULL_CHECK
    uint8_t *buff = p_bme280->buffer;
    bme280_calib_table *calib = p_bme280->p_calib;

    i2c_read_block(p_bme280->fd, BME280_REG_DIG_T1, 26, buff);

    calib->dig_T1 = (uint16_t) ((((uint16_t) ((uint8_t) buff[1])) << 8) | buff[0]);
    calib->dig_T2 = (int16_t) ((((int16_t) ((int8_t) buff[3])) << 8) | buff[2]);
    calib->dig_T3 = (int16_t) ((((int16_t) ((int8_t) buff[5])) << 8) | buff[4]);
    calib->dig_P1 = (uint16_t) ((((uint16_t) ((uint8_t) buff[7])) << 8) | buff[6]);
    calib->dig_P2 = (int16_t) ((((int16_t) ((int8_t) buff[9])) << 8) | buff[8]);
    calib->dig_P3 = (int16_t) ((((int16_t) ((int8_t) buff[11])) << 8) | buff[10]);
    calib->dig_P4 = (int16_t) ((((int16_t) ((int8_t) buff[13])) << 8) | buff[12]);
    calib->dig_P5 = (int16_t) ((((int16_t) ((int8_t) buff[15])) << 8) | buff[14]);
    calib->dig_P6 = (int16_t) ((((int16_t) ((int8_t) buff[17])) << 8) | buff[16]);
    calib->dig_P7 = (int16_t) ((((int16_t) ((int8_t) buff[19])) << 8) | buff[18]);
    calib->dig_P8 = (int16_t) ((((int16_t) ((int8_t) buff[21])) << 8) | buff[20]);
    calib->dig_P9 = (int16_t) ((((int16_t) ((int8_t) buff[23])) << 8) | buff[22]);
    calib->dig_H1 = buff[25];

    i2c_read_block(p_bme280->fd, BME280_REG_DIG_H2, 7, buff);

    calib->dig_H2 = (int16_t) ((((int16_t) ((int8_t) buff[1])) << 8) | buff[0]);
    calib->dig_H3 = buff[2];
    calib->dig_H4 = (int16_t) ((((int16_t) ((int8_t) buff[3])) << 4)
            | (((uint8_t) 0x0F) & buff[4]));
    calib->dig_H5 = (int16_t) ((((int16_t) ((int8_t) buff[5])) << 4) | (buff[4] >> 4));
    calib->dig_H6 = (int8_t) buff[6];

    return U8_SUCCESS;
}

uint8_t bme280_read_burst_tph()
{
    NULL_CHECK
    uint8_t *buff = p_bme280->buffer;
    bme280_uncomp_meas *meas = p_bme280->p_uncomp_meas;
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
    bme280_compensate_temp();
    bme280_compensate_press();
    bme280_compensate_humid();

    return U8_SUCCESS;
}

uint8_t bme280_read_temp()
{
    NULL_CHECK
    uint8_t *buff = p_bme280->buffer;
    bme280_uncomp_meas *meas = p_bme280->p_uncomp_meas;
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
    uint8_t *buff = p_bme280->buffer;
    bme280_uncomp_meas *meas = p_bme280->p_uncomp_meas;
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
    uint8_t *buff = p_bme280->buffer;
    bme280_uncomp_meas *meas = p_bme280->p_uncomp_meas;
    if (i2c_read_block(p_bme280->fd, BME280_REG_HUMID, 2, buff) == ERROR)
    {
        return U8_ERROR;
    }
    meas->hmsb = buff[0];
    meas->hlsb = buff[1];
    return unpack_humid();
}

uint8_t bme280_set_powermode(uint8_t mode)
{
    NULL_CHECK
    uint8_t ctrl_val = 0;
    uint8_t prev_mode = 0;
    uint8_t ctrl_hum_pre = 0;
    uint8_t conf_pre = 0;
    uint8_t common = 0;

    if (mode <= BME280_NORMAL_MODE)
    {
        ctrl_val = p_bme280->ctrl_meas_reg;
        ctrl_val = set_bit_slice(ctrl_val, 0x03, 0, mode);
        prev_mode = bme280_get_powermode();

        if (prev_mode != BME280_SLEEP_MODE)
        {
            bme280_soft_rst();
            bme280_delay(3);

            conf_pre = p_bme280->config_reg;
            i2c_write_reg(p_bme280->fd, BME280_REG_CONF, conf_pre);

            ctrl_hum_pre = p_bme280->ctrl_humid_reg;
            i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

            i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, ctrl_val);
        }
        else
        {
            i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, ctrl_val);
        }

        i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &common);
        p_bme280->ctrl_meas_reg = common;

        i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &common);
        p_bme280->ctrl_humid_reg = common;

        i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &common);
        p_bme280->config_reg = common;
    }
    else
    {
        return U8_ERROR;
    }
    return U8_SUCCESS;
}

uint8_t bme280_get_powermode()
{
    NULL_CHECK
    uint8_t mode = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &mode);
    p_bme280->power_mode = get_bit_slice(mode, 0x03, 0);
    return p_bme280->power_mode;
}

uint8_t bme280_set_temp_oversample(uint8_t rate)
{
    NULL_CHECK
    uint8_t common = 0;
    uint8_t mode_prev = 0;
    uint8_t ctrl_hum_pre = 0;
    uint8_t conf_pre = 0;

    common = p_bme280->ctrl_meas_reg;
    common = set_bit_slice(common, 0xE0, 5, rate);
    mode_prev = bme280_get_powermode();

    if (mode_prev != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);

        conf_pre = p_bme280->config_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, conf_pre);

        ctrl_hum_pre = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, common);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, common);
    }
    p_bme280->ovrsmpl_temp = rate;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &common);

    p_bme280->ctrl_meas_reg = common;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &common);

    p_bme280->ctrl_humid_reg = common;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &common);

    p_bme280->config_reg = common;

    return U8_SUCCESS;
}

uint8_t bme280_get_temp_oversample()
{
    NULL_CHECK
    uint8_t rate = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &rate);
    p_bme280->ovrsmpl_temp = get_bit_slice(rate, 0xE0, 5); // define macros
    return p_bme280->ovrsmpl_temp;
}

uint8_t bme280_set_press_oversample(uint8_t rate)
{
    NULL_CHECK
    uint8_t common = 0;
    uint8_t mode_prev = 0;
    uint8_t ctrl_hum_pre = 0;
    uint8_t conf_pre = 0;

    common = p_bme280->ctrl_meas_reg;
    common = set_bit_slice(common, 0x1C, 2, rate);
    mode_prev = bme280_get_powermode();

    if (mode_prev != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);

        conf_pre = p_bme280->config_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, conf_pre);

        ctrl_hum_pre = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, common);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, common);
    }
    p_bme280->ovrsmpl_press = rate;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &common);

    p_bme280->ctrl_meas_reg = common;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &common);

    p_bme280->ctrl_humid_reg = common;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &common);

    p_bme280->config_reg = common;

    return U8_SUCCESS;
}

uint8_t bme280_get_press_oversample()
{
    NULL_CHECK
    uint8_t rate = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &rate);
    p_bme280->ovrsmpl_press = get_bit_slice(rate, 0x1C, 2);
    return p_bme280->ovrsmpl_press;
}

uint8_t bme280_set_humid_oversample(uint8_t rate)
{
    NULL_CHECK
    uint8_t common = 0;
    uint8_t ctrl_pre = 0;
    uint8_t conf_pre = 0;
    uint8_t mode_prev = 0;

    common = p_bme280->ctrl_humid_reg;
    common = set_bit_slice(common, 0x07, 0, rate);
    mode_prev = bme280_get_powermode();

    if (mode_prev != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);

        conf_pre = p_bme280->config_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, conf_pre);
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, common);

        ctrl_pre = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, common);
        ctrl_pre = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, ctrl_pre);
    }
    p_bme280->ovrsmpl_humid = rate;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &common);

    p_bme280->ctrl_meas_reg = common;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &common);

    p_bme280->ctrl_humid_reg = common;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &common);

    p_bme280->config_reg = common;

    return U8_SUCCESS;
}

uint8_t bme280_get_humid_oversample()
{
    NULL_CHECK
    uint8_t rate = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &rate);
    p_bme280->ovrsmpl_humid = get_bit_slice(rate, 0x07, 0);
    return p_bme280->ovrsmpl_humid;
}

uint8_t bme280_set_standby_durn(uint8_t ms)
{
    NULL_CHECK
    uint8_t common = 0;
    uint8_t ctrl_pre = 0;
    uint8_t mode_prev = 0;
    uint8_t ctrl_hum_pre = 0;

    common = p_bme280->config_reg;
    common = set_bit_slice(common, 0xE0, 5, ms);
    mode_prev = bme280_get_powermode();

    if (mode_prev != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);

        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, common);

        ctrl_hum_pre = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        ctrl_pre = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, common);
    }
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &common);
    p_bme280->ctrl_meas_reg = common;

    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &common);
    p_bme280->ctrl_humid_reg = common;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &common);

    p_bme280->config_reg = common;

    return U8_SUCCESS;
}

uint8_t bme280_get_standby_durn()
{
    NULL_CHECK
    uint8_t durn = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &durn);
    p_bme280->standby_durn = get_bit_slice(durn, 0xE0, 5);
    return p_bme280->standby_durn;
}

uint8_t bme280_set_filter(uint8_t coef)
{
    NULL_CHECK
    uint8_t common = 0;
    uint8_t ctrl_pre = 0;
    uint8_t mode_prev = 0;
    uint8_t ctrl_hum_pre = 0;

    common = p_bme280->config_reg;
    common = set_bit_slice(common, 0x1C, 2, coef);
    mode_prev = bme280_get_powermode();

    if (mode_prev != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);

        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, common);

        ctrl_hum_pre = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        ctrl_pre = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, common);
    }
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &common);
    p_bme280->ctrl_meas_reg = common;

    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &common);
    p_bme280->ctrl_humid_reg = common;

    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &common);
    p_bme280->config_reg = common;

    return U8_SUCCESS;
}

uint8_t bme280_get_filter()
{
    NULL_CHECK
    uint8_t coeff = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &coeff);
    p_bme280->filter = get_bit_slice(coeff, 0x1C, 2);
    return p_bme280->filter;
}

uint8_t bme280_compensate_temp()
{
    NULL_CHECK
    int32_t adc_T = p_bme280->p_uncomp_meas->temperature;
    bme280_calib_table *calib = p_bme280->p_calib;
    /*int32_t vx1 = 0;
     int32_t vx2 = 0;
     int32_t temperature = 0;
     int32_t tmp = p_bme280->p_uncomp_meas->temperature;
     bme280_calib_table *calib = p_bme280->p_calib;


     vx1 =
     ((((tmp >> 3) - ((int32_t) calib->dig_T1 << 1))) * ((int32_t) calib->dig_T2)) >> 11;

     vx2 = (((((tmp >> 4) - ((int32_t) calib->dig_T1))
     * ((tmp >> 4) - ((int32_t) calib->dig_T1)))
     >> 12)
     * ((int32_t) calib->dig_T3));
     calib->t_fine = vx1 + vx2;
     temperature = (calib->t_fine * 5 + 128) >> 8;

     p_bme280->comp_temp = temperature;*/
    double var1, var2;
    var1 =
            (((double) adc_T) / 16384.0 - ((double) calib->dig_T1) / 1024.0) * ((double) calib->dig_T2);
    var2 =
            ((((double) adc_T) / 131072.0 - ((double) calib->dig_T1) / 8192.0) * (((double) adc_T)
                    / 131072.0
                                                                                  - ((double) calib->dig_T1) / 8192.0))
            * ((double) calib->dig_T3);
    p_bme280->p_uncomp_meas->t_fine = (int64_t) (var1 + var2);
    p_bme280->comp_temp = (var1 + var2) / 5120.0;

    return U8_SUCCESS;
}

uint8_t bme280_compensate_press()
{
    NULL_CHECK
    int32_t adc_P = p_bme280->p_uncomp_meas->pressure;
    bme280_calib_table *calib = p_bme280->p_calib;
    /*32 BIT
     int32_t vx1 = 0;int32_t vx2 = 0;uint32_t pressure = 0;
     int32_t tmp = p_bme280->p_uncomp_meas->pressure;
     bme280_calib_table *calib = p_bme280->p_calib;
     vx1 = (((int32_t) calib->t_fine) >> 1) - 64000;
     vx2 = (((vx1 >> 2) * (vx1 >> 2)) >> 11) * ((int32_t) calib->dig_P6);
     vx2 = vx2 + ((vx1 * ((int32_t) calib->dig_P5)) << 1);
     vx2 = (vx2 >> 2) + (((int32_t) calib->dig_P4) << 16);
     vx1 = (((((int32_t) calib->dig_P3) * (((vx1 >> 2) * (vx1 >> 2)) >> 13)) >> 3) + ((((int32_t) calib->dig_P2)
     * vx1)>> 1))>> 18;vx1 = ((32768 + vx1) * ((int32_t) calib->dig_P1)) >> 15;
     pressure = (((uint32_t) (1048576 - tmp) - (vx2 >> 12))) * 3125;
     if (pressure < 0x80000000)
     {
     if (vx1 != 0)
     {
     pressure = (pressure << 1) / ((uint32_t) vx1);
     }
     else
     {
     return U8_ERROR;
     }
     }
     else if (vx1 != 0)
     {
     pressure = (pressure / (uint32_t) vx1) << 1;
     }
     else
     {
     return U8_ERROR;
     }
     vx1 = (((int32_t) calib->dig_P9) * ((int32_t) (((pressure >> 3) * (pressure >> 3))
     >> 13)))
     >> 12;
     vx2 = (((int32_t) (pressure >> 2)) * ((int32_t) calib->dig_P8)) >> 13;
     pressure = (uint32_t) ((int32_t) pressure + ((vx1 + vx2 + calib->dig_P7) >> 4));

     p_bme280->comp_press = pressure;*/
    /*64 BIT
     int64_t var1, var2, p;
     var1 = ((int64_t) p_bme280->p_uncomp_meas->t_fine) - 128000;
     var2 = var1 * var1 * (int64_t) calib->dig_P6;
     var2 = var2 + ((var1 * (int64_t) calib->dig_P5) << 17);
     var2 = var2 + (((int64_t) calib->dig_P4) << 35);
     var1 = ((var1 * var1 * (int64_t) calib->dig_P3) >> 8) + ((var1
     * (int64_t) calib->dig_P2)
     << 12);
     var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) calib->dig_P1) >> 33;
     if (var1 == 0)
     {
     p_bme280->comp_press = 0;
     return U8_ERROR; // avoid exception caused by division by zero
     }
     p = 1048576 - adc_P;
     p = (((p << 31) - var2) * 3125) / var1;
     var1 = (((int64_t) calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
     var2 = (((int64_t) calib->dig_P8) * p) >> 19;
     p = ((p + var1 + var2) >> 8) + (((int64_t) calib->dig_P7) << 4);
     p_bme280->comp_press = (uint32_t) p;
     */

    double var1, var2, p;
    var1 = ((double) p_bme280->p_uncomp_meas->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) calib->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double) calib->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) calib->dig_P4) * 65536.0);
    var1 = (((double) calib->dig_P3) * var1 * var1 / 524288.0 + ((double) calib->dig_P2)
            * var1)
           / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) calib->dig_P1);
    if (var1 == 0.0)
    {
        return U8_ERROR;
    }
    p = 1048576.0 - (double) adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) calib->dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double) calib->dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double) calib->dig_P7)) / 16.0;
    p_bme280->comp_press = p / 100.0;

    return U8_SUCCESS;
}

uint8_t bme280_compensate_humid()
{
    NULL_CHECK
    int32_t adc_H = p_bme280->p_uncomp_meas->humidity;
    bme280_calib_table *calib = p_bme280->p_calib;
    /*    int32_t tmp = p_bme280->p_uncomp_meas->humidity;
     bme280_calib_table *calib = p_bme280->p_calib;
     int32_t vx1 = 0;

     vx1 = (calib->t_fine - ((int32_t) 76800));
     vx1 = (((((tmp << 14) - (((int32_t) calib->dig_H4) << 20)
     - (((int32_t) calib->dig_H5) * vx1))
     + ((int32_t) 16384))
     >> 15)
     * (((((((vx1 * ((int32_t) calib->dig_H6)) >> 10) * (((vx1
     * ((int32_t) calib->dig_H3))
     >> 11)
     + ((int32_t) 32768)))
     >> 10)
     + ((int32_t) 2097152))
     * ((int32_t) calib->dig_H2)
     + 8192)
     >> 14));
     vx1 = (vx1 - (((((vx1 >> 15) * (vx1 >> 15)) >> 7) * ((int32_t) calib->dig_H1)) >> 4));
     vx1 = (vx1 < 0 ? 0 : vx1);
     vx1 = (vx1 > 419430400 ? 419430400 : vx1);

     p_bme280->comp_humid = (uint32_t) (vx1 >> 12);*/
    double var_H;
    var_H = (((double) p_bme280->p_uncomp_meas->t_fine) - 76800.0);
    var_H = (adc_H
            - (((double) calib->dig_H4) * 64.0 + ((double) calib->dig_H5) / 16384.0
                    * var_H))
            * (((double) calib->dig_H2) / 65536.0 * (1.0
                    + ((double) calib->dig_H6) / 67108864.0 * var_H
                      * (1.0 + ((double) calib->dig_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((double) calib->dig_H1) * var_H / 524288.0);
    if (var_H > 100.0) var_H = 100.0;
    else if (var_H < 0.0) var_H = 0.0;
    p_bme280->comp_humid = var_H;

    return U8_SUCCESS;
}

double bme280_temp()
{
    NULL_CHECK
    return p_bme280->comp_temp;
}

double bme280_press()
{
    NULL_CHECK
    return p_bme280->comp_press;
}

double bme280_humid()
{
    NULL_CHECK
    return p_bme280->comp_humid;
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

void bme280_delay(uint8_t ms)
{
    struct timespec tv, tvr;
    tv.tv_sec = 0;
    tv.tv_nsec = ms * 1000000L;
    nanosleep(&tv, &tvr);
}

uint8_t unpack_press()
{
    NULL_CHECK
    bme280_uncomp_meas *meas = p_bme280->p_uncomp_meas;
    meas->pressure = (int32_t) ((((uint32_t) meas->pmsb) << 12)
            | (((uint32_t) meas->plsb) << 4) | ((uint32_t) meas->pxsb >> 4));
    return U8_SUCCESS;
}

uint8_t unpack_temp()
{
    NULL_CHECK
    bme280_uncomp_meas *meas = p_bme280->p_uncomp_meas;
    meas->temperature = (int32_t) ((((uint32_t) meas->tmsb) << 12)
            | (((uint32_t) meas->tlsb) << 4) | ((uint32_t) meas->txsb >> 4));
    return U8_SUCCESS;
}

uint8_t unpack_humid()
{
    NULL_CHECK
    bme280_uncomp_meas *meas = p_bme280->p_uncomp_meas;
    meas->humidity = (int32_t) ((((uint32_t) meas->hmsb) << 8) | ((uint32_t) meas->hlsb));
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

uint8_t set_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos, uint8_t val)
{
    return (regvar & ~mask) | ((val << pos) & mask);
}

uint8_t get_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos)
{
    return (regvar & mask) >> pos;
}
