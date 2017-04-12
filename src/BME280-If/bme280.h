#ifndef SRC_BME280_IF_BME280_H_
#define SRC_BME280_IF_BME280_H_

#include <stdint.h>

#define U8_ERROR (0xFF)
#define U8_SUCCESS (0x00)

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
/**\name    POWER MODE DEFINITIONS  */
/***************************************************/
/* Sensor Specific constants */
#define BME280_SLEEP_MODE (0x00)
#define BME280_FORCED_MODE (0x01)
#define BME280_NORMAL_MODE (0x03)
/****************************************************/
/**\name    STANDBY DEFINITIONS  */
/***************************************************/
#define BME280_STANDBY_TIME_1_MS (0x00)
#define BME280_STANDBY_TIME_63_MS (0x01)
#define BME280_STANDBY_TIME_125_MS (0x02)
#define BME280_STANDBY_TIME_250_MS (0x03)
#define BME280_STANDBY_TIME_500_MS (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS (0x06)
#define BME280_STANDBY_TIME_20_MS (0x07)
/****************************************************/
/**\name    OVER SAMPLING DEFINITIONS  */
/***************************************************/
#define BME280_OVERSAMP_SKIPPED (0x00)
#define BME280_OVERSAMP_1X (0x01)
#define BME280_OVERSAMP_2X (0x02)
#define BME280_OVERSAMP_4X (0x03)
#define BME280_OVERSAMP_8X (0x04)
#define BME280_OVERSAMP_16X (0x05)
/****************************************************/
/**\name    FILTER DEFINITIONS  */
/***************************************************/
#define BME280_FILTER_COEFF_OFF (0x00)
#define BME280_FILTER_COEFF_2 (0x01)
#define BME280_FILTER_COEFF_4 (0x02)
#define BME280_FILTER_COEFF_8 (0x03)
#define BME280_FILTER_COEFF_16 (0x04)
/****************************************************/
/**\name    DELAY DEFINITIONS  */
/***************************************************/
#define T_INIT_MAX (20)
/* 20/16 = 1.25 ms */
#define T_MEASURE_PER_OSRS_MAX (37)
/* 37/16 = 2.3125 ms*/

#define T_SETUP_PRESSURE_MAX (10)
/* 10/16 = 0.625 ms */

#define T_SETUP_HUMIDITY_MAX (10)

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;

} bme280_calib_table;

typedef struct
{
    uint8_t pmsb;
    uint8_t plsb;
    uint8_t pxsb;
    uint8_t tmsb;
    uint8_t tlsb;
    uint8_t txsb;
    uint8_t hmsb;
    uint8_t hlsb;
    int32_t temperature;
    int32_t pressure;
    int32_t humidity;
    int64_t t_fine;

} bme280_uncomp_meas;

typedef struct
{
    int fd;
    uint8_t buffer[32];
    bme280_calib_table *p_calib;
    bme280_uncomp_meas *p_uncomp_meas;
    uint8_t power_mode;
    uint8_t filter;
    uint8_t standby_durn;
    uint8_t ovrsmpl_temp;
    uint8_t ovrsmpl_press;
    uint8_t ovrsmpl_humid;
    /*int32_t comp_temp;
     uint32_t comp_press;
     uint32_t comp_humid;*/
    double comp_temp;
    double comp_press;
    double comp_humid;
    uint8_t ctrl_humid_reg;
    uint8_t ctrl_meas_reg;
    uint8_t config_reg;

} bme280;

void printbin(uint32_t i);

/**
 * Initialize the bme280.
 * THIS FUNCTION MUST BE CALLED
 * BEFORE ANY OTHER BME280 INTERACTIONS!
 */
uint8_t bme280_init(const char *dev, bme280 *inst);
/**
 * Close the bme280.
 * IF SO THIS FUNCTION MUST BE CALLED AT LAST!
 */
uint8_t bme280_deinit();
/**
 * Read calibration table.
 */
uint8_t bme280_read_calib();
/**
 * Read not-compensated temperature, pressure
 * and humidity all in once (burst).
 */
uint8_t bme280_read_burst_tph();
/**
 * Read not-compensated temperature.
 */
uint8_t bme280_read_temp();
/**
 * Read not-compensated pressure.
 */
uint8_t bme280_read_press();
/**
 * Read not-compensated humidity.
 */
uint8_t bme280_read_humid();
/**
 * Set power mode.
 */
uint8_t bme280_set_powermode(uint8_t mode);
/**
 * Get power mode.
 */
uint8_t bme280_get_powermode();
/**
 * Set temperature oversample rate.
 */
uint8_t bme280_set_temp_oversample(uint8_t rate);
/**
 * Get temperature oversample rate.
 */
uint8_t bme280_get_temp_oversample();
/**
 * Set pressure oversample rate.
 */
uint8_t bme280_set_press_oversample(uint8_t rate);
/**
 * Get pressure oversample rate.
 */
uint8_t bme280_get_press_oversample();
/**
 * Set humidity oversample rate.
 */
uint8_t bme280_set_humid_oversample(uint8_t rate);
/**
 * Get humidity oversample rate.
 */
uint8_t bme280_get_humid_oversample();
/**
 * Set standby duration.
 */
uint8_t bme280_set_standby_durn(uint8_t ms);
/**
 * Get standby duration.
 */
uint8_t bme280_get_standby_durn();
/**
 * Set filter coefficient.
 */
uint8_t bme280_set_filter(uint8_t coef);
/**
 * Get filter coefficient.
 */
uint8_t bme280_get_filter();
/**
 * Compensate temperature.
 */
uint8_t bme280_compensate_temp();
/**
 * Compensate pressure.
 */
uint8_t bme280_compensate_press();
/**
 * Compensate humidity.
 */
uint8_t bme280_compensate_humid();
/**
 * Get compensated temperature as double.
 */
double bme280_temp();
/**
 * Get compensated pressure as double.
 */
double bme280_press();
/**
 * Get compensated humidity as double.
 */
double bme280_humid();
/**
 * Perform a soft reset.
 */
uint8_t bme280_soft_rst();
/**
 * Wait amount of ms doing nothing.
 */
void bme280_delay(uint8_t ms);

#endif /* SRC_BME280_IF_BME280_H_ */
