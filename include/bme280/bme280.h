/*
 Author: Julian P. Becht

 This program is free software; you can redistribute it and/or
 modify it.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 */

#pragma once

#include "util/types.h"

#define BME280_ERROR (U8_MAX)
#define BME280_SUCCESS (U8_MIN)

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
// Power modes
#define BME280_SLEEP_MODE (0x00)
#define BME280_FORCED_MODE (0x01)
#define BME280_NORMAL_MODE (0x03)
// Standby times
#define BME280_STANDBY_TIME_1_MS (0x00)
#define BME280_STANDBY_TIME_63_MS (0x01)
#define BME280_STANDBY_TIME_125_MS (0x02)
#define BME280_STANDBY_TIME_250_MS (0x03)
#define BME280_STANDBY_TIME_500_MS (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS (0x06)
#define BME280_STANDBY_TIME_20_MS (0x07)
// Oversampling ratios
#define BME280_OVERSAMP_SKIPPED (0x00)
#define BME280_OVERSAMP_1X (0x01)
#define BME280_OVERSAMP_2X (0x02)
#define BME280_OVERSAMP_4X (0x03)
#define BME280_OVERSAMP_8X (0x04)
#define BME280_OVERSAMP_16X (0x05)
// Filter coefficients
#define BME280_FILTER_COEFF_OFF (0x00)
#define BME280_FILTER_COEFF_2 (0x01)
#define BME280_FILTER_COEFF_4 (0x02)
#define BME280_FILTER_COEFF_8 (0x03)
#define BME280_FILTER_COEFF_16 (0x04)

/**
 * BME280 calibration table
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
 * BME280 uncompensated measurements
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
 * Initialize the bme280.
 * THIS FUNCTION MUST BE CALLED
 * BEFORE ANY OTHER BME280 INTERACTIONS!
 */
u8_t BME280_init(const char* dev, bme280* inst);

/**
 * Close the bme280.
 * IF SO THIS FUNCTION MUST BE CALLED AT LAST!
 */
void BME280_deinit(bme280* inst);

/**
 * Read calibration table.
 */
u8_t BME280_read_calib(bme280* inst);

/**
 * Read not-compensated temperature, pressure
 * and humidity all in once (burst).
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
 * Get compensated temperature as f64_t.
 */
f64_t BME280_temp(bme280* inst);

/**
 * Get compensated pressure as f64_t.
 */
f64_t BME280_press(bme280* inst);

/**
 * Get compensated humidity as f64_t.
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
