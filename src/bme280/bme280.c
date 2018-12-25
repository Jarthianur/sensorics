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

#include "bme280/bme280.h"

#include <stddef.h>
#include <time.h>
#include <unistd.h>

#include "i2c/i2cif.h"

void    unpack_press(bme280*);
void    unpack_temp(bme280*);
void    unpack_humid(bme280*);
uint8_t set_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos, uint8_t val);
uint8_t get_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos);

uint8_t BME280_init(const char* dev, bme280* inst)
{
    if ((inst->fd = I2C_init_dev(dev, BME280_ADDRESS)) == -1)
    {
        return U8_ERROR;
    }
    return BME280_read_calib(inst);
}

void BME280_deinit(bme280* inst)
{
    I2C_close_dev(inst->fd);
}

uint8_t BME280_read_calib(bme280* inst)
{
    uint8_t*            buff  = inst->buffer;
    bme280_calib_table* calib = &inst->p_calib;

    I2C_read_block(inst->fd, BME280_REG_DIG_T1, 26, buff);

    calib->dig_T1 = (uint16_t)((((uint16_t)((uint8_t) buff[1])) << 8) | buff[0]);
    calib->dig_T2 = (int16_t)((((int16_t)((int8_t) buff[3])) << 8) | buff[2]);
    calib->dig_T3 = (int16_t)((((int16_t)((int8_t) buff[5])) << 8) | buff[4]);
    calib->dig_P1 = (uint16_t)((((uint16_t)((uint8_t) buff[7])) << 8) | buff[6]);
    calib->dig_P2 = (int16_t)((((int16_t)((int8_t) buff[9])) << 8) | buff[8]);
    calib->dig_P3 = (int16_t)((((int16_t)((int8_t) buff[11])) << 8) | buff[10]);
    calib->dig_P4 = (int16_t)((((int16_t)((int8_t) buff[13])) << 8) | buff[12]);
    calib->dig_P5 = (int16_t)((((int16_t)((int8_t) buff[15])) << 8) | buff[14]);
    calib->dig_P6 = (int16_t)((((int16_t)((int8_t) buff[17])) << 8) | buff[16]);
    calib->dig_P7 = (int16_t)((((int16_t)((int8_t) buff[19])) << 8) | buff[18]);
    calib->dig_P8 = (int16_t)((((int16_t)((int8_t) buff[21])) << 8) | buff[20]);
    calib->dig_P9 = (int16_t)((((int16_t)((int8_t) buff[23])) << 8) | buff[22]);
    calib->dig_H1 = buff[25];

    I2C_read_block(inst->fd, BME280_REG_DIG_H2, 7, buff);

    calib->dig_H2 = (int16_t)((((int16_t)((int8_t) buff[1])) << 8) | buff[0]);
    calib->dig_H3 = buff[2];
    calib->dig_H4 = (int16_t)((((int16_t)((int8_t) buff[3])) << 4) | (((uint8_t) 0x0F) & buff[4]));
    calib->dig_H5 = (int16_t)((((int16_t)((int8_t) buff[5])) << 4) | (buff[4] >> 4));
    calib->dig_H6 = (int8_t) buff[6];

    return U8_SUCCESS;
}

uint8_t BME280_read_burst_tph(bme280* inst)
{
    uint8_t*            buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    if (I2C_read_block(inst->fd, BME280_REG_MEAS, 8, buff) == -1)
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
    unpack_press(inst);
    unpack_temp(inst);
    unpack_humid(inst);
    BME280_compensate_temp(inst);
    BME280_compensate_press(inst);
    BME280_compensate_humid(inst);

    return U8_SUCCESS;
}

uint8_t BME280_read_temp(bme280* inst)
{
    uint8_t*            buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    if (I2C_read_block(inst->fd, BME280_REG_TEMP, 3, buff) == -1)
    {
        return U8_ERROR;
    }
    meas->tmsb = buff[0];
    meas->tlsb = buff[1];
    meas->txsb = buff[2];
    unpack_temp(inst);
    return U8_SUCCESS;
}

uint8_t BME280_read_press(bme280* inst)
{
    uint8_t*            buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    if (I2C_read_block(inst->fd, BME280_REG_PRESS, 3, buff) == -1)
    {
        return U8_ERROR;
    }
    meas->pmsb = buff[0];
    meas->plsb = buff[1];
    meas->pxsb = buff[2];
    unpack_press(inst);
    return U8_SUCCESS;
}

uint8_t BME280_read_humid(bme280* inst)
{
    uint8_t*            buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    if (I2C_read_block(inst->fd, BME280_REG_HUMID, 2, buff) == -1)
    {
        return U8_ERROR;
    }
    meas->hmsb = buff[0];
    meas->hlsb = buff[1];
    unpack_humid(inst);
    return U8_SUCCESS;
}

uint8_t BME280_set_powermode(bme280* inst, uint8_t mode)
{
    uint8_t ctrl_val     = 0;
    uint8_t prev_mode    = 0;
    uint8_t ctrl_hum_pre = 0;
    uint8_t conf_pre     = 0;
    uint8_t common       = 0;

    if (mode <= BME280_NORMAL_MODE)
    {
        ctrl_val  = inst->ctrl_meas_reg;
        ctrl_val  = set_bit_slice(ctrl_val, 0x03, 0, mode);
        prev_mode = BME280_get_powermode(inst);

        if (prev_mode != BME280_SLEEP_MODE)
        {
            BME280_soft_rst(inst);
            BME280_delay(3);

            conf_pre = inst->config_reg;
            I2C_write_reg(inst->fd, BME280_REG_CONF, conf_pre);

            ctrl_hum_pre = inst->ctrl_humid_reg;
            I2C_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

            I2C_write_reg(inst->fd, BME280_REG_CTRL, ctrl_val);
        }
        else
        {
            I2C_write_reg(inst->fd, BME280_REG_CTRL, ctrl_val);
        }

        I2C_read_reg(inst->fd, BME280_REG_CTRL, &common);
        inst->ctrl_meas_reg = common;

        I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
        inst->ctrl_humid_reg = common;

        I2C_read_reg(inst->fd, BME280_REG_CONF, &common);
        inst->config_reg = common;
    }
    else
    {
        return U8_ERROR;
    }
    return U8_SUCCESS;
}

uint8_t BME280_get_powermode(bme280* inst)
{
    uint8_t mode = 0;
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &mode);
    inst->power_mode = get_bit_slice(mode, 0x03, 0);
    return inst->power_mode;
}

uint8_t BME280_set_temp_oversample(bme280* inst, uint8_t rate)
{
    uint8_t common       = 0;
    uint8_t mode_prev    = 0;
    uint8_t ctrl_hum_pre = 0;
    uint8_t conf_pre     = 0;

    common    = inst->ctrl_meas_reg;
    common    = set_bit_slice(common, 0xE0, 5, rate);
    mode_prev = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);

        conf_pre = inst->config_reg;
        I2C_write_reg(inst->fd, BME280_REG_CONF, conf_pre);

        ctrl_hum_pre = inst->ctrl_humid_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        I2C_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    else
    {
        I2C_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    inst->ovrsmpl_temp = rate;
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &common);

    inst->ctrl_meas_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);

    inst->ctrl_humid_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CONF, &common);

    inst->config_reg = common;

    return U8_SUCCESS;
}

uint8_t BME280_get_temp_oversample(bme280* inst)
{
    uint8_t rate = 0;
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &rate);
    inst->ovrsmpl_temp = get_bit_slice(rate, 0xE0, 5);  // define macros
    return inst->ovrsmpl_temp;
}

uint8_t BME280_set_press_oversample(bme280* inst, uint8_t rate)
{
    uint8_t common       = 0;
    uint8_t mode_prev    = 0;
    uint8_t ctrl_hum_pre = 0;
    uint8_t conf_pre     = 0;

    common    = inst->ctrl_meas_reg;
    common    = set_bit_slice(common, 0x1C, 2, rate);
    mode_prev = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);

        conf_pre = inst->config_reg;
        I2C_write_reg(inst->fd, BME280_REG_CONF, conf_pre);

        ctrl_hum_pre = inst->ctrl_humid_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        I2C_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    else
    {
        I2C_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    inst->ovrsmpl_press = rate;
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &common);

    inst->ctrl_meas_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);

    inst->ctrl_humid_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CONF, &common);

    inst->config_reg = common;

    return U8_SUCCESS;
}

uint8_t BME280_get_press_oversample(bme280* inst)
{
    uint8_t rate = 0;
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &rate);
    inst->ovrsmpl_press = get_bit_slice(rate, 0x1C, 2);
    return inst->ovrsmpl_press;
}

uint8_t BME280_set_humid_oversample(bme280* inst, uint8_t rate)
{
    uint8_t common    = 0;
    uint8_t ctrl_pre  = 0;
    uint8_t conf_pre  = 0;
    uint8_t mode_prev = 0;

    common    = inst->ctrl_humid_reg;
    common    = set_bit_slice(common, 0x07, 0, rate);
    mode_prev = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);

        conf_pre = inst->config_reg;
        I2C_write_reg(inst->fd, BME280_REG_CONF, conf_pre);
        I2C_write_reg(inst->fd, BME280_REG_CTRL_HUMID, common);

        ctrl_pre = inst->ctrl_meas_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        I2C_write_reg(inst->fd, BME280_REG_CTRL_HUMID, common);
        ctrl_pre = inst->ctrl_meas_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    inst->ovrsmpl_humid = rate;
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &common);

    inst->ctrl_meas_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);

    inst->ctrl_humid_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CONF, &common);

    inst->config_reg = common;

    return U8_SUCCESS;
}

uint8_t BME280_get_humid_oversample(bme280* inst)
{
    uint8_t rate = 0;
    I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &rate);
    inst->ovrsmpl_humid = get_bit_slice(rate, 0x07, 0);
    return inst->ovrsmpl_humid;
}

uint8_t BME280_set_standby_durn(bme280* inst, uint8_t ms)
{
    uint8_t common       = 0;
    uint8_t ctrl_pre     = 0;
    uint8_t mode_prev    = 0;
    uint8_t ctrl_hum_pre = 0;

    common    = inst->config_reg;
    common    = set_bit_slice(common, 0xE0, 5, ms);
    mode_prev = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);

        I2C_write_reg(inst->fd, BME280_REG_CONF, common);

        ctrl_hum_pre = inst->ctrl_humid_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        ctrl_pre = inst->ctrl_meas_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        I2C_write_reg(inst->fd, BME280_REG_CONF, common);
    }
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &common);
    inst->ctrl_meas_reg = common;

    I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
    inst->ctrl_humid_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CONF, &common);

    inst->config_reg = common;

    return U8_SUCCESS;
}

uint8_t BME280_get_standby_durn(bme280* inst)
{
    uint8_t durn = 0;
    I2C_read_reg(inst->fd, BME280_REG_CONF, &durn);
    inst->standby_durn = get_bit_slice(durn, 0xE0, 5);
    return inst->standby_durn;
}

uint8_t BME280_set_filter(bme280* inst, uint8_t coef)
{
    uint8_t common       = 0;
    uint8_t ctrl_pre     = 0;
    uint8_t mode_prev    = 0;
    uint8_t ctrl_hum_pre = 0;

    common    = inst->config_reg;
    common    = set_bit_slice(common, 0x1C, 2, coef);
    mode_prev = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);

        I2C_write_reg(inst->fd, BME280_REG_CONF, common);

        ctrl_hum_pre = inst->ctrl_humid_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);

        ctrl_pre = inst->ctrl_meas_reg;
        I2C_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        I2C_write_reg(inst->fd, BME280_REG_CONF, common);
    }
    I2C_read_reg(inst->fd, BME280_REG_CTRL, &common);
    inst->ctrl_meas_reg = common;

    I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
    inst->ctrl_humid_reg = common;

    I2C_read_reg(inst->fd, BME280_REG_CONF, &common);
    inst->config_reg = common;

    return U8_SUCCESS;
}

uint8_t BME280_get_filter(bme280* inst)
{
    uint8_t coeff = 0;
    I2C_read_reg(inst->fd, BME280_REG_CONF, &coeff);
    inst->filter = get_bit_slice(coeff, 0x1C, 2);
    return inst->filter;
}

uint8_t BME280_compensate_temp(bme280* inst)
{
    int32_t             adc_T = inst->p_uncomp_meas.temperature;
    bme280_calib_table* calib = &inst->p_calib;
    double              var1, var2;
    var1 =
        (((double) adc_T) / 16384.0 - ((double) calib->dig_T1) / 1024.0) * ((double) calib->dig_T2);
    var2 = ((((double) adc_T) / 131072.0 - ((double) calib->dig_T1) / 8192.0) *
            (((double) adc_T) / 131072.0 - ((double) calib->dig_T1) / 8192.0)) *
           ((double) calib->dig_T3);
    inst->p_uncomp_meas.t_fine = (int64_t)(var1 + var2);
    inst->comp_temp            = (var1 + var2) / 5120.0;

    return U8_SUCCESS;
}

uint8_t BME280_compensate_press(bme280* inst)
{
    int32_t             adc_P = inst->p_uncomp_meas.pressure;
    bme280_calib_table* calib = &inst->p_calib;
    double              var1, var2, p;
    var1 = ((double) inst->p_uncomp_meas.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) calib->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double) calib->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) calib->dig_P4) * 65536.0);
    var1 = (((double) calib->dig_P3) * var1 * var1 / 524288.0 + ((double) calib->dig_P2) * var1) /
           524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) calib->dig_P1);
    if (var1 == 0.0)
    {
        return U8_ERROR;
    }
    p                = 1048576.0 - (double) adc_P;
    p                = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1             = ((double) calib->dig_P9) * p * p / 2147483648.0;
    var2             = p * ((double) calib->dig_P8) / 32768.0;
    p                = p + (var1 + var2 + ((double) calib->dig_P7)) / 16.0;
    inst->comp_press = p / 100.0;

    return U8_SUCCESS;
}

uint8_t BME280_compensate_humid(bme280* inst)
{
    int32_t             adc_H = inst->p_uncomp_meas.humidity;
    bme280_calib_table* calib = &inst->p_calib;
    double              var_H;
    var_H = (((double) inst->p_uncomp_meas.t_fine) - 76800.0);
    var_H =
        (adc_H - (((double) calib->dig_H4) * 64.0 + ((double) calib->dig_H5) / 16384.0 * var_H)) *
        (((double) calib->dig_H2) / 65536.0 *
         (1.0 + ((double) calib->dig_H6) / 67108864.0 * var_H *
                    (1.0 + ((double) calib->dig_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((double) calib->dig_H1) * var_H / 524288.0);
    if (var_H > 100.0)
        var_H = 100.0;
    else if (var_H < 0.0)
        var_H = 0.0;
    inst->comp_humid = var_H;

    return U8_SUCCESS;
}

double BME280_temp(bme280* inst)
{
    return inst->comp_temp;
}

double BME280_press(bme280* inst)
{
    return inst->comp_press;
}

double BME280_humid(bme280* inst)
{
    return inst->comp_humid;
}

uint8_t BME280_soft_rst(bme280* inst)
{
    if (I2C_write_reg(inst->fd, BME280_REG_SOFTRST, BME280_RST) == -1)
    {
        return U8_ERROR;
    }
    else
    {
        return U8_SUCCESS;
    }
}

void BME280_delay(uint8_t ms)
{
    struct timespec tv, tvr;
    tv.tv_sec  = 0;
    tv.tv_nsec = ms * 1000000L;
    nanosleep(&tv, &tvr);
}

void unpack_press(bme280* inst)
{
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    meas->pressure = (int32_t)((((uint32_t) meas->pmsb) << 12) | (((uint32_t) meas->plsb) << 4) |
                               ((uint32_t) meas->pxsb >> 4));
}

void unpack_temp(bme280* inst)
{
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    meas->temperature = (int32_t)((((uint32_t) meas->tmsb) << 12) | (((uint32_t) meas->tlsb) << 4) |
                                  ((uint32_t) meas->txsb >> 4));
}

void unpack_humid(bme280* inst)
{
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    meas->humidity           = (int32_t)((((uint32_t) meas->hmsb) << 8) | ((uint32_t) meas->hlsb));
}

uint8_t set_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos, uint8_t val)
{
    return (regvar & ~mask) | ((val << pos) & mask);
}

uint8_t get_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos)
{
    return (regvar & mask) >> pos;
}
