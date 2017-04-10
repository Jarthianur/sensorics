#include "bme280.h"
#include "i2cif.h"
#include <stddef.h>
#include <time.h>

#define NULL_CHECK if (check_null() == (U8_ERROR)) return (U8_ERROR);

int32_t unpack_press();
int32_t unpack_temp();
int32_t unpack_humid();
uint8_t check_null();
uint8_t set_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos, uint8_t val);
uint8_t get_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos);

static bme280* p_bme280 = NULL;

uint8_t bme280_init(const char* dev, bme280* inst)
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
    uint8_t* buff = p_bme280->buffer;
    bme280_calib_table* calib = p_bme280->p_calib;

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
    NULL_CHECK
    uint8_t v_mode_uint8_tr = 0;
    uint8_t v_prev_pow_mode_uint8_t = 0;
    uint8_t v_pre_ctrl_hum_value_uint8_t = 0;
    uint8_t v_pre_config_value_uint8_t = 0;
    uint8_t v_data_uint8_t = 0;

    if (mode <= BME280_NORMAL_MODE)
    {
        v_mode_uint8_tr = p_bme280->ctrl_meas_reg;
        v_mode_uint8_tr = set_bit_slice(v_mode_uint8_tr, 0x03, 0, mode);

        v_prev_pow_mode_uint8_t = bme280_powermode();
        if (v_prev_pow_mode_uint8_t != BME280_SLEEP_MODE)
        {
            bme280_soft_rst();
            bme280_delay(3);
            /* write previous value of
             configuration register*/
            v_pre_config_value_uint8_t = p_bme280->config_reg;
            i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_pre_config_value_uint8_t);

            /* write previous value of
             humidity oversampling*/
            v_pre_ctrl_hum_value_uint8_t = p_bme280->ctrl_humid_reg;
            i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID,
                          v_pre_ctrl_hum_value_uint8_t);

            /* write previous and updated value of
             control measurement register*/
            i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, v_mode_uint8_tr);
        }
        else
        {
            i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, v_mode_uint8_tr);
        }
        /* read the control measurement register value*/
        i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);
        p_bme280->ctrl_meas_reg = v_data_uint8_t;

        /* read the control humidity register value*/
        i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &v_data_uint8_t);
        p_bme280->ctrl_humid_reg = v_data_uint8_t;

        /* read the config register value*/
        i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);
        p_bme280->config_reg = v_data_uint8_t;
    }
    else
    {
        return U8_ERROR;
    }
    return U8_SUCCESS;
}

uint8_t bme280_powermode()
{
    NULL_CHECK
    uint8_t v_mode_uint8_tr = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_mode_uint8_tr);
    p_bme280->power_mode = get_bit_slice(v_mode_uint8_tr, 0x03, 0); // define macros
    return p_bme280->power_mode;
}

uint8_t bme280_temp_oversample(uint8_t rate)
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    uint8_t v_prev_pow_mode_uint8_t = 0;
    uint8_t v_pre_ctrl_hum_value_uint8_t = 0;
    uint8_t v_pre_config_value_uint8_t = 0;

    v_data_uint8_t = p_bme280->ctrl_meas_reg;
    v_data_uint8_t = set_bit_slice(v_data_uint8_t, 0xE0, 5, rate);
    v_prev_pow_mode_uint8_t = bme280_powermode();
    if (v_prev_pow_mode_uint8_t != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);
        /* write previous value
         of configuration register*/
        v_pre_config_value_uint8_t = p_bme280->config_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_pre_config_value_uint8_t);

        /* write previous value
         of humidity oversampling*/
        v_pre_ctrl_hum_value_uint8_t = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, v_pre_ctrl_hum_value_uint8_t);

        /* write previous and updated value
         of configuration register*/
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, v_data_uint8_t);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, v_data_uint8_t);
    }
    p_bme280->ovrsmpl_temp = rate;
    /* read the control measurement register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);
    p_bme280->ctrl_meas_reg = v_data_uint8_t;

    /* read the control humidity register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &v_data_uint8_t);
    p_bme280->ctrl_humid_reg = v_data_uint8_t;

    /* read the control
     configuration register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);

    p_bme280->config_reg = v_data_uint8_t;
    return U8_SUCCESS;
}

uint8_t bme280_temp_oversample()
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);
    p_bme280->ovrsmpl_temp = get_bit_slice(v_data_uint8_t, 0xE0, 5); // define macros
    return p_bme280->ovrsmpl_temp;
}

uint8_t bme280_press_oversample(uint8_t rate)
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    uint8_t v_prev_pow_mode_uint8_t = 0;
    uint8_t v_pre_ctrl_hum_value_uint8_t = 0;
    uint8_t v_pre_config_value_uint8_t = 0;

    v_data_uint8_t = p_bme280->ctrl_meas_reg;
    v_data_uint8_t = set_bit_slice(v_data_uint8_t, 0x1C, 2, rate);

    v_prev_pow_mode_uint8_t = bme280_powermode();
    if (v_prev_pow_mode_uint8_t != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);

        /* write previous value of
         configuration register*/
        v_pre_config_value_uint8_t = p_bme280->config_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_pre_config_value_uint8_t);

        /* write previous value of
         humidity oversampling*/
        v_pre_ctrl_hum_value_uint8_t = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, v_pre_ctrl_hum_value_uint8_t);

        /* write previous and updated value of
         control measurement register*/
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, v_data_uint8_t);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, v_data_uint8_t);
    }
    p_bme280->ovrsmpl_press = rate;
    /* read the control measurement register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);
    p_bme280->ctrl_meas_reg = v_data_uint8_t;

    /* read the control humidity register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &v_data_uint8_t);
    p_bme280->ctrl_humid_reg = v_data_uint8_t;
    /* read the control
     configuration register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);
    p_bme280->config_reg = v_data_uint8_t;

    return U8_SUCCESS;
}

uint8_t bme280_press_oversample()
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);
    p_bme280->ovrsmpl_press = get_bit_slice(v_data_uint8_t, 0x1C, 2); //TODO define macros
    return p_bme280->ovrsmpl_press;
}

uint8_t bme280_humid_oversample(uint8_t rate)
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    uint8_t pre_ctrl_meas_value = 0;
    uint8_t v_pre_config_value_uint8_t = 0;
    uint8_t v_prev_pow_mode_uint8_t = 0;

    /* write humidity oversampling*/
    v_data_uint8_t = p_bme280->ctrl_humid_reg;
    v_data_uint8_t = set_bit_slice(v_data_uint8_t, 0x07, 0, rate); //TODO DEFINE MACROS

    v_prev_pow_mode_uint8_t = bme280_powermode();
    if (v_prev_pow_mode_uint8_t != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);
        /* write previous value of
         configuration register*/
        v_pre_config_value_uint8_t = p_bme280->config_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_pre_config_value_uint8_t);

        /* write the value of control humidity*/
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, v_data_uint8_t);

        /* write previous value of
         control measurement register*/
        pre_ctrl_meas_value = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, pre_ctrl_meas_value);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, v_data_uint8_t);
        /* Control humidity write will effective only
         after the control measurement register*/
        pre_ctrl_meas_value = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, pre_ctrl_meas_value);
    }
    p_bme280->ovrsmpl_humid = rate;
    /* read the control measurement register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);

    p_bme280->ctrl_meas_reg = v_data_uint8_t;
    /* read the control humidity register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &v_data_uint8_t);

    p_bme280->ctrl_humid_reg = v_data_uint8_t;
    /* read the control configuration register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);

    p_bme280->config_reg = v_data_uint8_t;

    return U8_SUCCESS;
}

uint8_t bme280_humid_oversample()
{
    NULL_CHECK
    uint8_t val = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &val);
    p_bme280->ovrsmpl_humid = get_bit_slice(val, BME280_REG_CTRL_HUMID);
    return p_bme280->ovrsmpl_humid;
}

uint8_t bme280_standby_durn(uint8_t ms)
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    uint8_t pre_ctrl_meas_value = 0;
    uint8_t v_prev_pow_mode_uint8_t = 0;
    uint8_t v_pre_ctrl_hum_value_uint8_t = 0;

    v_data_uint8_t = p_bme280->config_reg;
    v_data_uint8_t = set_bit_slice(v_data_uint8_t, 0xE0, 5, ms);
    v_prev_pow_mode_uint8_t = bme280_powermode();

    if (v_prev_pow_mode_uint8_t != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);
        /* write previous and updated value of
         configuration register*/
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_data_uint8_t);

        /* write previous value of
         humidity oversampling*/
        v_pre_ctrl_hum_value_uint8_t = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, v_pre_ctrl_hum_value_uint8_t);

        /* write previous value of control
         measurement register*/
        pre_ctrl_meas_value = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, pre_ctrl_meas_value);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_data_uint8_t);
    }
    /* read the control measurement register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);
    p_bme280->ctrl_meas_reg = v_data_uint8_t;

    /* read the control humidity register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &v_data_uint8_t);
    p_bme280->ctrl_humid_reg = v_data_uint8_t;

    /* read the configuration register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);

    p_bme280->config_reg = v_data_uint8_t;
    return U8_SUCCESS;
}

uint8_t bme280_standby_durn()
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);
    p_bme280->standby_durn = get_bit_slice(v_data_uint8_t, 0xE0, 5); // TODO define macros
    return p_bme280->standby_durn;
}

uint8_t bme280_filter(uint8_t coef)
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    uint8_t pre_ctrl_meas_value = 0;
    uint8_t v_prev_pow_mode_uint8_t = 0;
    uint8_t v_pre_ctrl_hum_value_uint8_t = 0;

    v_data_uint8_t = p_bme280->config_reg;
    v_data_uint8_t = set_bit_slice(v_data_uint8_t, 0x1C, 2, coef);
    v_prev_pow_mode_uint8_t = bme280_powermode();
    if (v_prev_pow_mode_uint8_t != BME280_SLEEP_MODE)
    {
        bme280_soft_rst();
        bme280_delay(3);
        /* write previous and updated value of
         configuration register*/
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_data_uint8_t);

        /* write previous value of
         humidity oversampling*/
        v_pre_ctrl_hum_value_uint8_t = p_bme280->ctrl_humid_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, v_pre_ctrl_hum_value_uint8_t);

        /* write previous value of
         control measurement register*/
        pre_ctrl_meas_value = p_bme280->ctrl_meas_reg;
        i2c_write_reg(p_bme280->fd, BME280_REG_CTRL, pre_ctrl_meas_value);
    }
    else
    {
        i2c_write_reg(p_bme280->fd, BME280_REG_CONF, v_data_uint8_t);
    }
    /* read the control measurement register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL, &v_data_uint8_t);
    p_bme280->ctrl_meas_reg = v_data_uint8_t;

    /* read the control humidity register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CTRL_HUMID, &v_data_uint8_t);
    p_bme280->ctrl_humid_reg = v_data_uint8_t;

    /* read the configuration register value*/
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);
    p_bme280->config_reg = v_data_uint8_t;
    return U8_SUCCESS;
}

uint8_t bme280_filter()
{
    NULL_CHECK
    uint8_t v_data_uint8_t = 0;
    i2c_read_reg(p_bme280->fd, BME280_REG_CONF, &v_data_uint8_t);
    p_bme280->filter = get_bit_slice(v_data_uint8_t, 0x1C, 2); //TODO define macros
    return p_bme280->filter;
}

uint8_t bme280_compensate_temp()
{
    NULL_CHECK
    int32_t vx1 = 0;
    int32_t vx2 = 0;
    int32_t temperature = 0;
    int32_t tmp = p_bme280->p_uncomp_meas->temperature;
    bme280_calib_table* calib = p_bme280->p_calib;

    /* calculate x1*/
    vx1 =
            ((((tmp >> 3) - ((int32_t) calib->dig_T1 << 1))) * ((int32_t) calib->dig_T2)) >> 11;
    /* calculate x2*/
    vx2 = (((((tmp >> 4) - ((int32_t) calib->dig_T1))
            * ((tmp >> 4) - ((int32_t) calib->dig_T1)))
            >> 12)
           * ((int32_t) calib->dig_T3))
          >> 14;
    /* calculate t_fine*/
    calib->t_fine = vx1 + vx2;
    /* calculate temperature*/
    temperature = (calib->t_fine * 5 + 128) >> 8;

    p_bme280->comp_temp = temperature;
    return U8_SUCCESS;
}

uint8_t bme280_compensate_press()
{
    NULL_CHECK
    int32_t vx1 = 0;
    int32_t vx2 = 0;
    uint32_t pressure = 0;
    int32_t tmp = p_bme280->p_uncomp_meas->pressure;
    bme280_calib_table* calib = p_bme280->p_calib;

    /* calculate x1*/
    vx1 = (((int32_t) calib->t_fine) >> 1) - (int32_t) 64000;
    /* calculate x2*/
    vx2 = (((vx1 >> 2) * (vx1 >> 2)) >> 11) * ((int32_t) calib->dig_P6);
    /* calculate x2*/
    vx2 = vx2 + ((vx1 * ((int32_t) calib->dig_P5)) << 1);
    /* calculate x2*/
    vx2 = (vx2 >> 2) + (((int32_t) calib->dig_P4) << 16);
    /* calculate x1*/
    vx1 =
            (((calib->dig_P3 * (((vx1 >> 2) * (vx1 >> 2)) >> 13)) >> 3) + ((((int32_t) calib->dig_P2)
                    * vx1)
                                                                           >> 1))
            >> 18;
    /* calculate x1*/
    vx1 = ((((32768 + vx1)) * ((int32_t) calib->dig_P1)) >> 15);
    /* calculate pressure*/
    pressure = (((uint32_t) (((int32_t) 1048576) - tmp) - (vx2 >> 12))) * 3125;
    if (pressure < 0x80000000)
    /* Avoid exception caused by division by zero */
    if (vx1 != 0) pressure = (pressure << 1) / ((uint32_t) vx1);
    else return U8_ERROR;
    else
    /* Avoid exception caused by division by zero */
    if (vx1 != 0) pressure = (pressure / (uint32_t) vx1) * 2;
    else return U8_ERROR;

    vx1 = (((int32_t) calib->dig_P9) * ((int32_t) (((pressure >> 3) * (pressure >> 3))
            >> 13)))
          >> 12;
    vx2 = (((int32_t) (pressure >> 2)) * ((int32_t) calib->dig_P8)) >> 13;
    pressure = (uint32_t) ((int32_t) pressure + ((vx1 + vx2 + calib->dig_P7) >> 4));

    p_bme280->comp_press = pressure;
    return U8_SUCCESS;
}

uint8_t bme280_compensate_humid()
{
    NULL_CHECK
    int32_t tmp = p_bme280->p_uncomp_meas->humidity;
    bme280_calib_table* calib = p_bme280->p_calib;
    int32_t vx1 = 0;

    /* calculate x1*/
    vx1 = (calib->t_fine - ((int32_t) 76800));
    /* calculate x1*/
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

    p_bme280->comp_humid = (uint32_t) (vx1 >> 12);
    return U8_SUCCESS;
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

uint8_t set_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos, uint8_t val)
{
    return (regvar & ~mask) | ((val << pos) & mask);
}

uint8_t get_bit_slice(uint8_t regvar, uint8_t mask, uint8_t pos)
{
    return (regvar & mask) >> pos;
}

