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

#define _POSIX_C_SOURCE 200809L

#include <time.h>
#include <unistd.h>

#include "util/types.h"

#ifdef I2C_API
#    define BUS_init(dev, addr) I2C_init_dev(dev, addr)
#    define BUS_close(fd) I2C_close_dev(fd)
#    define BUS_write_reg(fd, reg, data) I2C_write_reg(fd, reg, data)
#    define BUS_read_reg(fd, reg, data) I2C_read_reg(fd, reg, data)
#    define BUS_read_block(fd, reg, bytes, data) I2C_read_block(fd, reg, bytes, data)
#else
#    error No BUS API given!
#endif

#define BME280_ERROR (U8_MAX)
#define BME280_SUCCESS (U8_MIN)

/// (register) addresses
#define BME280_ADDRESS (0x76)

#define BME280_REG_DIG_T1 (0x88)
#define BME280_REG_DIG_T2 (0x8A)
#define BME280_REG_DIG_T3 (0x8C)
#define BME280_REG_DIG_P1 (0x8E)
#define BME280_REG_DIG_P2 (0x90)
#define BME280_REG_DIG_P3 (0x92)
#define BME280_REG_DIG_P4 (0x94)
#define BME280_REG_DIG_P5 (0x96)
#define BME280_REG_DIG_P6 (0x98)
#define BME280_REG_DIG_P7 (0x9A)
#define BME280_REG_DIG_P8 (0x9C)
#define BME280_REG_DIG_P9 (0x9E)
#define BME280_REG_DIG_H1 (0xA1)
#define BME280_REG_DIG_H2 (0xE1)
#define BME280_REG_DIG_H3 (0xE3)
#define BME280_REG_DIG_H4 (0xE4)
#define BME280_REG_DIG_H5 (0xE5)
#define BME280_REG_DIG_H6 (0xE7)
#define BME280_REG_CHIPID (0xD0)
#define BME280_REG_VERSION (0xD1)
#define BME280_REG_SOFTRST (0xE0)
#define BME280_RST (0xB6)
#define BME280_REG_CAL26 (0xE1)
#define BME280_REG_CTRL_HUMID (0xF2)
#define BME280_REG_CTRL (0xF4)
#define BME280_REG_CONF (0xF5)
#define BME280_REG_PRESS (0xF7)
#define BME280_REG_TEMP (0xFA)
#define BME280_REG_HUMID (0xFD)
#define BME280_REG_MEAS (BME280_REG_PRESS)
// power modes
#define BME280_SLEEP_MODE (0x00)
#define BME280_FORCED_MODE (0x01)
#define BME280_NORMAL_MODE (0x03)
// standby times
#define BME280_STANDBY_TIME_1_MS (0x00)
#define BME280_STANDBY_TIME_10_MS (0x06)
#define BME280_STANDBY_TIME_20_MS (0x07)
#define BME280_STANDBY_TIME_63_MS (0x01)
#define BME280_STANDBY_TIME_125_MS (0x02)
#define BME280_STANDBY_TIME_250_MS (0x03)
#define BME280_STANDBY_TIME_500_MS (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
// oversampling ratios
#define BME280_OVERSAMP_SKIPPED (0x00)
#define BME280_OVERSAMP_1X (0x01)
#define BME280_OVERSAMP_2X (0x02)
#define BME280_OVERSAMP_4X (0x03)
#define BME280_OVERSAMP_8X (0x04)
#define BME280_OVERSAMP_16X (0x05)
// filter coefficients
#define BME280_FILTER_COEFF_OFF (0x00)
#define BME280_FILTER_COEFF_2 (0x01)
#define BME280_FILTER_COEFF_4 (0x02)
#define BME280_FILTER_COEFF_8 (0x03)
#define BME280_FILTER_COEFF_16 (0x04)

/**
 * calibration table
 */
typedef struct
{
    u16_t dig_T1;
    s16_t dig_T2;
    s16_t dig_T3;
    u16_t dig_P1;
    s16_t dig_P2;
    s16_t dig_P3;
    s16_t dig_P4;
    s16_t dig_P5;
    s16_t dig_P6;
    s16_t dig_P7;
    s16_t dig_P8;
    s16_t dig_P9;
    u8_t  dig_H1;
    s16_t dig_H2;
    u8_t  dig_H3;
    s16_t dig_H4;
    s16_t dig_H5;
    s8_t  dig_H6;

} bme280_calib_table;

/**
 * uncompensated measurements
 */
typedef struct
{
    u8_t  pmsb;
    u8_t  plsb;
    u8_t  pxsb;
    u8_t  tmsb;
    u8_t  tlsb;
    u8_t  txsb;
    u8_t  hmsb;
    u8_t  hlsb;
    s32_t temperature;
    s32_t pressure;
    s32_t humidity;
    s64_t t_fine;

} bme280_uncomp_meas;

/**
 * BME280
 */
typedef struct
{
    int                fd;
    u8_t               buffer[32];
    bme280_calib_table p_calib;
    bme280_uncomp_meas p_uncomp_meas;
    u8_t               power_mode;
    u8_t               filter;
    u8_t               standby_durn;
    u8_t               ovrsmpl_temp;
    u8_t               ovrsmpl_press;
    u8_t               ovrsmpl_humid;
    f64_t              comp_temp;
    f64_t              comp_press;
    f64_t              comp_humid;
    u8_t               ctrl_humid_reg;
    u8_t               ctrl_meas_reg;
    u8_t               config_reg;

} bme280;

/**
 * Initialize bme280.
 */
u8_t BME280_init(const char* dev, bme280* inst);

/**
 * Close bme280.
 */
void BME280_deinit(bme280* inst);

/**
 * Read calibration table.
 */
u8_t BME280_read_calib(bme280* inst);

/**
 * Read not-compensated temperature, pressure
 * and humidity all at once (burst).
 */
u8_t BME280_read_burst_tph(bme280* inst);

/**
 * Read not-compensated temperature.
 */
u8_t BME280_read_temp(bme280* inst);

/**
 * Read not-compensated pressure.
 */
u8_t BME280_read_press(bme280* inst);

/**
 * Read not-compensated humidity.
 */
u8_t BME280_read_humid(bme280* inst);

/**
 * Set power mode.
 */
u8_t BME280_set_powermode(bme280* inst, u8_t mode);

/**
 * Get power mode.
 */
u8_t BME280_get_powermode(bme280* inst);

/**
 * Set temperature oversample rate.
 */
u8_t BME280_set_temp_oversample(bme280* inst, u8_t rate);

/**
 * Get temperature oversample rate.
 */
u8_t BME280_get_temp_oversample(bme280* inst);

/**
 * Set pressure oversample rate.
 */
u8_t BME280_set_press_oversample(bme280* inst, u8_t rate);

/**
 * Get pressure oversample rate.
 */
u8_t BME280_get_press_oversample(bme280* inst);

/**
 * Set humidity oversample rate.
 */
u8_t BME280_set_humid_oversample(bme280* inst, u8_t rate);

/**
 * Get humidity oversample rate.
 */
u8_t BME280_get_humid_oversample(bme280* inst);

/**
 * Set standby duration.
 */
u8_t BME280_set_standby_durn(bme280* inst, u8_t ms);

/**
 * Get standby duration.
 */
u8_t BME280_get_standby_durn(bme280* inst);

/**
 * Set filter coefficient.
 */
u8_t BME280_set_filter(bme280* inst, u8_t coef);

/**
 * Get filter coefficient.
 */
u8_t BME280_get_filter(bme280* inst);

/**
 * Compensate temperature.
 */
u8_t BME280_compensate_temp(bme280* inst);

/**
 * Compensate pressure.
 */
u8_t BME280_compensate_press(bme280* inst);

/**
 * Compensate humidity.
 */
u8_t BME280_compensate_humid(bme280* inst);

/**
 * Get compensated temperature.
 */
f64_t BME280_temp(bme280* inst);

/**
 * Get compensated pressure.
 */
f64_t BME280_press(bme280* inst);

/**
 * Get compensated humidity.
 */
f64_t BME280_humid(bme280* inst);

/**
 * Perform a soft reset.
 */
u8_t BME280_soft_rst(bme280* inst);

/**
 * Wait amount of ms doing nothing.
 */
void BME280_delay(u8_t ms);

/**
 * @brief Unpack pressure value.
 * @private
 */
void BME280_unpack_pressure(bme280*);

/**
 * @brief Unpack temperature value.
 * @private
 */
void BME280_unpack_temperature(bme280*);

/**
 * @brief Unpack humidity value.
 * @private
 */
void BME280_unpack_humidity(bme280*);

/**
 * @brief Get bitslice for value.
 * @private
 */
u8_t BME280_bit_slice(u8_t regvar, u8_t mask, u8_t pos, u8_t val);

/**
 * @brief Get value of bitslice.
 * @private
 */
u8_t BME280_get_bit_slice(u8_t regvar, u8_t mask, u8_t pos);

u8_t BME280_init(const char* dev, bme280* inst)
{
    if ((inst->fd = BUS_init(dev, BME280_ADDRESS)) == -1)
    {
        return BME280_ERROR;
    }
    return BME280_read_calib(inst);
}

void BME280_deinit(bme280* inst)
{
    BUS_close(inst->fd);
}

u8_t BME280_read_calib(bme280* inst)
{
    u8_t*               buff  = inst->buffer;
    bme280_calib_table* calib = &inst->p_calib;
    BUS_read_block(inst->fd, BME280_REG_DIG_T1, 26, buff);
    calib->dig_T1 = (u16_t)((((u16_t)((u8_t) buff[1])) << 8) | buff[0]);
    calib->dig_T2 = (s16_t)((((s16_t)((s8_t) buff[3])) << 8) | buff[2]);
    calib->dig_T3 = (s16_t)((((s16_t)((s8_t) buff[5])) << 8) | buff[4]);
    calib->dig_P1 = (u16_t)((((u16_t)((u8_t) buff[7])) << 8) | buff[6]);
    calib->dig_P2 = (s16_t)((((s16_t)((s8_t) buff[9])) << 8) | buff[8]);
    calib->dig_P3 = (s16_t)((((s16_t)((s8_t) buff[11])) << 8) | buff[10]);
    calib->dig_P4 = (s16_t)((((s16_t)((s8_t) buff[13])) << 8) | buff[12]);
    calib->dig_P5 = (s16_t)((((s16_t)((s8_t) buff[15])) << 8) | buff[14]);
    calib->dig_P6 = (s16_t)((((s16_t)((s8_t) buff[17])) << 8) | buff[16]);
    calib->dig_P7 = (s16_t)((((s16_t)((s8_t) buff[19])) << 8) | buff[18]);
    calib->dig_P8 = (s16_t)((((s16_t)((s8_t) buff[21])) << 8) | buff[20]);
    calib->dig_P9 = (s16_t)((((s16_t)((s8_t) buff[23])) << 8) | buff[22]);
    calib->dig_H1 = buff[25];
    BUS_read_block(inst->fd, BME280_REG_DIG_H2, 7, buff);
    calib->dig_H2 = (s16_t)((((s16_t)((s8_t) buff[1])) << 8) | buff[0]);
    calib->dig_H3 = buff[2];
    calib->dig_H4 = (s16_t)((((s16_t)((s8_t) buff[3])) << 4) | (((u8_t) 0x0F) & buff[4]));
    calib->dig_H5 = (s16_t)((((s16_t)((s8_t) buff[5])) << 4) | (buff[4] >> 4));
    calib->dig_H6 = (s8_t) buff[6];

    return BME280_SUCCESS;
}

u8_t BME280_read_burst_tph(bme280* inst)
{
    u8_t*               buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;

    if (BUS_read_block(inst->fd, BME280_REG_MEAS, 8, buff) == -1)
    {
        return BME280_ERROR;
    }
    meas->pmsb = buff[0];
    meas->plsb = buff[1];
    meas->pxsb = buff[2];
    meas->tmsb = buff[3];
    meas->tlsb = buff[4];
    meas->txsb = buff[5];
    meas->hmsb = buff[6];
    meas->hlsb = buff[7];
    BME280_unpack_pressure(inst);
    BME280_unpack_temperature(inst);
    BME280_unpack_humidity(inst);
    BME280_compensate_temp(inst);
    BME280_compensate_press(inst);
    BME280_compensate_humid(inst);

    return BME280_SUCCESS;
}

u8_t BME280_read_temp(bme280* inst)
{
    u8_t*               buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;

    if (BUS_read_block(inst->fd, BME280_REG_TEMP, 3, buff) == -1)
    {
        return BME280_ERROR;
    }
    meas->tmsb = buff[0];
    meas->tlsb = buff[1];
    meas->txsb = buff[2];
    BME280_unpack_temperature(inst);

    return BME280_SUCCESS;
}

u8_t BME280_read_press(bme280* inst)
{
    u8_t*               buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;

    if (BUS_read_block(inst->fd, BME280_REG_PRESS, 3, buff) == -1)
    {
        return BME280_ERROR;
    }
    meas->pmsb = buff[0];
    meas->plsb = buff[1];
    meas->pxsb = buff[2];
    BME280_unpack_pressure(inst);

    return BME280_SUCCESS;
}

u8_t BME280_read_humid(bme280* inst)
{
    u8_t*               buff = inst->buffer;
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;

    if (BUS_read_block(inst->fd, BME280_REG_HUMID, 2, buff) == -1)
    {
        return BME280_ERROR;
    }
    meas->hmsb = buff[0];
    meas->hlsb = buff[1];
    BME280_unpack_humidity(inst);

    return BME280_SUCCESS;
}

u8_t BME280_set_powermode(bme280* inst, u8_t mode)
{
    u8_t ctrl_val     = 0;
    u8_t prev_mode    = 0;
    u8_t ctrl_hum_pre = 0;
    u8_t conf_pre     = 0;
    u8_t common       = 0;

    if (mode <= BME280_NORMAL_MODE)
    {
        ctrl_val  = inst->ctrl_meas_reg;
        ctrl_val  = BME280_bit_slice(ctrl_val, 0x03, 0, mode);
        prev_mode = BME280_get_powermode(inst);

        if (prev_mode != BME280_SLEEP_MODE)
        {
            BME280_soft_rst(inst);
            BME280_delay(3);
            conf_pre = inst->config_reg;
            BUS_write_reg(inst->fd, BME280_REG_CONF, conf_pre);
            ctrl_hum_pre = inst->ctrl_humid_reg;
            BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);
            BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_val);
        }
        else
        {
            BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_val);
        }
        BUS_read_reg(inst->fd, BME280_REG_CTRL, &common);
        inst->ctrl_meas_reg = common;
        BUS_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
        inst->ctrl_humid_reg = common;
        BUS_read_reg(inst->fd, BME280_REG_CONF, &common);
        inst->config_reg = common;
    }
    else
    {
        return BME280_ERROR;
    }
    return BME280_SUCCESS;
}

u8_t BME280_get_powermode(bme280* inst)
{
    u8_t mode = 0;
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &mode);
    inst->power_mode = BME280_get_bit_slice(mode, 0x03, 0);

    return inst->power_mode;
}

u8_t BME280_set_temp_oversample(bme280* inst, u8_t rate)
{
    u8_t common       = 0;
    u8_t mode_prev    = 0;
    u8_t ctrl_hum_pre = 0;
    u8_t conf_pre     = 0;
    common            = inst->ctrl_meas_reg;
    common            = BME280_bit_slice(common, 0xE0, 5, rate);
    mode_prev         = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);
        conf_pre = inst->config_reg;
        BUS_write_reg(inst->fd, BME280_REG_CONF, conf_pre);
        ctrl_hum_pre = inst->ctrl_humid_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);
        BUS_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    else
    {
        BUS_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    inst->ovrsmpl_temp = rate;
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &common);
    inst->ctrl_meas_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
    inst->ctrl_humid_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CONF, &common);
    inst->config_reg = common;

    return BME280_SUCCESS;
}

u8_t BME280_get_temp_oversample(bme280* inst)
{
    u8_t rate = 0;
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &rate);
    inst->ovrsmpl_temp = BME280_get_bit_slice(rate, 0xE0, 5);

    return inst->ovrsmpl_temp;
}

u8_t BME280_set_press_oversample(bme280* inst, u8_t rate)
{
    u8_t common       = 0;
    u8_t mode_prev    = 0;
    u8_t ctrl_hum_pre = 0;
    u8_t conf_pre     = 0;
    common            = inst->ctrl_meas_reg;
    common            = BME280_bit_slice(common, 0x1C, 2, rate);
    mode_prev         = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);
        conf_pre = inst->config_reg;
        BUS_write_reg(inst->fd, BME280_REG_CONF, conf_pre);
        ctrl_hum_pre = inst->ctrl_humid_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);
        BUS_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    else
    {
        BUS_write_reg(inst->fd, BME280_REG_CTRL, common);
    }
    inst->ovrsmpl_press = rate;
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &common);
    inst->ctrl_meas_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
    inst->ctrl_humid_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CONF, &common);
    inst->config_reg = common;

    return BME280_SUCCESS;
}

u8_t BME280_get_press_oversample(bme280* inst)
{
    u8_t rate = 0;
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &rate);
    inst->ovrsmpl_press = BME280_get_bit_slice(rate, 0x1C, 2);

    return inst->ovrsmpl_press;
}

u8_t BME280_set_humid_oversample(bme280* inst, u8_t rate)
{
    u8_t common    = 0;
    u8_t ctrl_pre  = 0;
    u8_t conf_pre  = 0;
    u8_t mode_prev = 0;
    common         = inst->ctrl_humid_reg;
    common         = BME280_bit_slice(common, 0x07, 0, rate);
    mode_prev      = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);
        conf_pre = inst->config_reg;
        BUS_write_reg(inst->fd, BME280_REG_CONF, conf_pre);
        BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, common);
        ctrl_pre = inst->ctrl_meas_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, common);
        ctrl_pre = inst->ctrl_meas_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    inst->ovrsmpl_humid = rate;
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &common);
    inst->ctrl_meas_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
    inst->ctrl_humid_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CONF, &common);
    inst->config_reg = common;

    return BME280_SUCCESS;
}

u8_t BME280_get_humid_oversample(bme280* inst)
{
    u8_t rate = 0;
    BUS_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &rate);
    inst->ovrsmpl_humid = BME280_get_bit_slice(rate, 0x07, 0);

    return inst->ovrsmpl_humid;
}

u8_t BME280_set_standby_durn(bme280* inst, u8_t ms)
{
    u8_t common       = 0;
    u8_t ctrl_pre     = 0;
    u8_t mode_prev    = 0;
    u8_t ctrl_hum_pre = 0;
    common            = inst->config_reg;
    common            = BME280_bit_slice(common, 0xE0, 5, ms);
    mode_prev         = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);
        BUS_write_reg(inst->fd, BME280_REG_CONF, common);
        ctrl_hum_pre = inst->ctrl_humid_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);
        ctrl_pre = inst->ctrl_meas_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        BUS_write_reg(inst->fd, BME280_REG_CONF, common);
    }
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &common);
    inst->ctrl_meas_reg = common;
    I2C_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
    inst->ctrl_humid_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CONF, &common);
    inst->config_reg = common;

    return BME280_SUCCESS;
}

u8_t BME280_get_standby_durn(bme280* inst)
{
    u8_t durn = 0;
    BUS_read_reg(inst->fd, BME280_REG_CONF, &durn);
    inst->standby_durn = BME280_get_bit_slice(durn, 0xE0, 5);

    return inst->standby_durn;
}

u8_t BME280_set_filter(bme280* inst, u8_t coef)
{
    u8_t common       = 0;
    u8_t ctrl_pre     = 0;
    u8_t mode_prev    = 0;
    u8_t ctrl_hum_pre = 0;
    common            = inst->config_reg;
    common            = BME280_bit_slice(common, 0x1C, 2, coef);
    mode_prev         = BME280_get_powermode(inst);

    if (mode_prev != BME280_SLEEP_MODE)
    {
        BME280_soft_rst(inst);
        BME280_delay(3);
        BUS_write_reg(inst->fd, BME280_REG_CONF, common);
        ctrl_hum_pre = inst->ctrl_humid_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);
        ctrl_pre = inst->ctrl_meas_reg;
        BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_pre);
    }
    else
    {
        BUS_write_reg(inst->fd, BME280_REG_CONF, common);
    }
    BUS_read_reg(inst->fd, BME280_REG_CTRL, &common);
    inst->ctrl_meas_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
    inst->ctrl_humid_reg = common;
    BUS_read_reg(inst->fd, BME280_REG_CONF, &common);
    inst->config_reg = common;

    return BME280_SUCCESS;
}

u8_t BME280_get_filter(bme280* inst)
{
    u8_t coeff = 0;
    BUS_read_reg(inst->fd, BME280_REG_CONF, &coeff);
    inst->filter = BME280_get_bit_slice(coeff, 0x1C, 2);

    return inst->filter;
}

u8_t BME280_compensate_temp(bme280* inst)
{
    s32_t               adc_T = inst->p_uncomp_meas.temperature;
    bme280_calib_table* calib = &inst->p_calib;
    f64_t               var1, var2;
    var1 = (((f64_t) adc_T) / 16384.0 - ((f64_t) calib->dig_T1) / 1024.0) * ((f64_t) calib->dig_T2);
    var2 = ((((f64_t) adc_T) / 131072.0 - ((f64_t) calib->dig_T1) / 8192.0) *
            (((f64_t) adc_T) / 131072.0 - ((f64_t) calib->dig_T1) / 8192.0)) *
           ((f64_t) calib->dig_T3);
    inst->p_uncomp_meas.t_fine = (s64_t)(var1 + var2);
    inst->comp_temp            = (var1 + var2) / 5120.0;

    return BME280_SUCCESS;
}

u8_t BME280_compensate_press(bme280* inst)
{
    s32_t               adc_P = inst->p_uncomp_meas.pressure;
    bme280_calib_table* calib = &inst->p_calib;
    f64_t               var1, var2, p;
    var1 = ((f64_t) inst->p_uncomp_meas.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((f64_t) calib->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((f64_t) calib->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((f64_t) calib->dig_P4) * 65536.0);
    var1 = (((f64_t) calib->dig_P3) * var1 * var1 / 524288.0 + ((f64_t) calib->dig_P2) * var1) /
           524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((f64_t) calib->dig_P1);
    if (var1 == 0.0)
    {
        return BME280_ERROR;
    }
    p                = 1048576.0 - (f64_t) adc_P;
    p                = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1             = ((f64_t) calib->dig_P9) * p * p / 2147483648.0;
    var2             = p * ((f64_t) calib->dig_P8) / 32768.0;
    p                = p + (var1 + var2 + ((f64_t) calib->dig_P7)) / 16.0;
    inst->comp_press = p / 100.0;

    return BME280_SUCCESS;
}

u8_t BME280_compensate_humid(bme280* inst)
{
    s32_t               adc_H = inst->p_uncomp_meas.humidity;
    bme280_calib_table* calib = &inst->p_calib;
    f64_t               var_H;
    var_H = (((f64_t) inst->p_uncomp_meas.t_fine) - 76800.0);
    var_H = (adc_H - (((f64_t) calib->dig_H4) * 64.0 + ((f64_t) calib->dig_H5) / 16384.0 * var_H)) *
            (((f64_t) calib->dig_H2) / 65536.0 *
             (1.0 + ((f64_t) calib->dig_H6) / 67108864.0 * var_H *
                        (1.0 + ((f64_t) calib->dig_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((f64_t) calib->dig_H1) * var_H / 524288.0);
    if (var_H > 100.0)
        var_H = 100.0;
    else if (var_H < 0.0)
        var_H = 0.0;
    inst->comp_humid = var_H;

    return BME280_SUCCESS;
}

f64_t BME280_temp(bme280* inst)
{
    return inst->comp_temp;
}

f64_t BME280_press(bme280* inst)
{
    return inst->comp_press;
}

f64_t BME280_humid(bme280* inst)
{
    return inst->comp_humid;
}

u8_t BME280_soft_rst(bme280* inst)
{
    if (BUS_write_reg(inst->fd, BME280_REG_SOFTRST, BME280_RST) == -1)
    {
        return BME280_ERROR;
    }
    else
    {
        return BME280_SUCCESS;
    }
}

void BME280_delay(u8_t ms)
{
    struct timespec tv, tvr;
    tv.tv_sec  = 0;
    tv.tv_nsec = ms * 1000000L;
    nanosleep(&tv, &tvr);
}

void BME280_unpack_pressure(bme280* inst)
{
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    meas->pressure           = (s32_t)((((u32_t) meas->pmsb) << 12) | (((u32_t) meas->plsb) << 4) |
                             ((u32_t) meas->pxsb >> 4));
}

void BME280_unpack_temperature(bme280* inst)
{
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    meas->temperature        = (s32_t)((((u32_t) meas->tmsb) << 12) | (((u32_t) meas->tlsb) << 4) |
                                ((u32_t) meas->txsb >> 4));
}

void BME280_unpack_humidity(bme280* inst)
{
    bme280_uncomp_meas* meas = &inst->p_uncomp_meas;
    meas->humidity           = (s32_t)((((u32_t) meas->hmsb) << 8) | ((u32_t) meas->hlsb));
}

u8_t BME280_bit_slice(u8_t regvar, u8_t mask, u8_t pos, u8_t val)
{
    return (regvar & ~mask) | ((val << pos) & mask);
}

u8_t BME280_get_bit_slice(u8_t regvar, u8_t mask, u8_t pos)
{
    return (regvar & mask) >> pos;
}
