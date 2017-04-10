#ifndef BME280_IF_SRC_BME280_H_
#define BME280_IF_SRC_BME280_H_

#include <stdint.h>

#define U8_ERROR                 (0xFF)
#define U8_SUCCESS               (0x00)

#define BME280_ADDRESS           (0x76)

#define BME280_REG_DIG_T1        (0x88)
#define BME280_REG_DIG_T2        (0x8A)
#define BME280_REG_DIG_T3        (0x8C)
#define BME280_REG_DIG_P1        (0x8E)
#define BME280_REG_DIG_P2        (0x90)
#define BME280_REG_DIG_P3        (0x92)
#define BME280_REG_DIG_P4        (0x94)
#define BME280_REG_DIG_P5        (0x96)
#define BME280_REG_DIG_P6        (0x98)
#define BME280_REG_DIG_P7        (0x9A)
#define BME280_REG_DIG_P8        (0x9C)
#define BME280_REG_DIG_P9        (0x9E)
#define BME280_REG_DIG_H1        (0xA1)
#define BME280_REG_DIG_H2        (0xE1)
#define BME280_REG_DIG_H3        (0xE3)
#define BME280_REG_DIG_H4        (0xE4)
#define BME280_REG_DIG_H5        (0xE5)
#define BME280_REG_DIG_H6        (0xE7)
#define BME280_REG_CHIPID        (0xD0)
#define BME280_REG_VERSION       (0xD1)
#define BME280_REG_SOFTRST       (0xE0)
#define BME280_RST               (0xB6)
#define BME280_REG_CAL26         (0xE1)
#define BME280_REG_CTRL_HUMID    (0xF2)
#define BME280_REG_CTRL          (0xF4)
#define BME280_REG_CONF          (0xF5)
#define BME280_REG_PRESS         (0xF7)
#define BME280_REG_TEMP          (0xFA)
#define BME280_REG_HUMID         (0xFD)
#define BME280_REG_MEAS          (BME280_REG_PRESS)

/* numeric definitions */
#define BME280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (26)
#define BME280_HUMIDITY_CALIB_DATA_LENGTH       (7)
#define BME280_GEN_READ_WRITE_DATA_LENGTH       (1)
#define BME280_HUMIDITY_DATA_LENGTH             (2)
#define BME280_TEMPERATURE_DATA_LENGTH          (3)
#define BME280_PRESSURE_DATA_LENGTH             (3)
#define BME280_ALL_DATA_FRAME_LENGTH            (8)

/**\name    POWER MODE DEFINITIONS  */
/***************************************************/
/* Sensor Specific constants */
#define BME280_SLEEP_MODE                    (0x00)
#define BME280_FORCED_MODE                   (0x01)
#define BME280_NORMAL_MODE                   (0x03)
/****************************************************/
/**\name    STANDBY DEFINITIONS  */
/***************************************************/
#define BME280_STANDBY_TIME_1_MS              (0x00)
#define BME280_STANDBY_TIME_63_MS             (0x01)
#define BME280_STANDBY_TIME_125_MS            (0x02)
#define BME280_STANDBY_TIME_250_MS            (0x03)
#define BME280_STANDBY_TIME_500_MS            (0x04)
#define BME280_STANDBY_TIME_1000_MS           (0x05)
#define BME280_STANDBY_TIME_10_MS             (0x06)
#define BME280_STANDBY_TIME_20_MS             (0x07)
/****************************************************/
/**\name    OVER SAMPLING DEFINITIONS  */
/***************************************************/
#define BME280_OVERSAMP_SKIPPED          (0x00)
#define BME280_OVERSAMP_1X               (0x01)
#define BME280_OVERSAMP_2X               (0x02)
#define BME280_OVERSAMP_4X               (0x03)
#define BME280_OVERSAMP_8X               (0x04)
#define BME280_OVERSAMP_16X              (0x05)
/****************************************************/
/**\name    FILTER DEFINITIONS  */
/***************************************************/
#define BME280_FILTER_COEFF_OFF               (0x00)
#define BME280_FILTER_COEFF_2                 (0x01)
#define BME280_FILTER_COEFF_4                 (0x02)
#define BME280_FILTER_COEFF_8                 (0x03)
#define BME280_FILTER_COEFF_16                (0x04)
/****************************************************/
/**\name    DELAY DEFINITIONS  */
/***************************************************/
#define T_INIT_MAX                             (20)
/* 20/16 = 1.25 ms */
#define T_MEASURE_PER_OSRS_MAX                 (37)
/* 37/16 = 2.3125 ms*/

#define T_SETUP_PRESSURE_MAX                   (10)
/* 10/16 = 0.625 ms */

#define T_SETUP_HUMIDITY_MAX                   (10)
/* 10/16 = 0.625 ms */
/****************************************************/
/**\name    DEFINITIONS FOR ARRAY SIZE OF DATA   */
/***************************************************/
#define BME280_HUMIDITY_DATA_SIZE       (2)
#define BME280_TEMPERATURE_DATA_SIZE    (3)
#define BME280_PRESSURE_DATA_SIZE       (3)
#define BME280_DATA_FRAME_SIZE          (8)
/**< data frames includes temperature,
 pressure and humidity*/
#define BME280_CALIB_DATA_SIZE          (26)

#define BME280_TEMPERATURE_MSB_DATA     (0)
#define BME280_TEMPERATURE_LSB_DATA     (1)
#define BME280_TEMPERATURE_XLSB_DATA    (2)
#define BME280_PRESSURE_MSB_DATA        (0)
#define BME280_PRESSURE_LSB_DATA        (1)
#define BME280_PRESSURE_XLSB_DATA       (2)
#define BME280_HUMIDITY_MSB_DATA        (0)
#define BME280_HUMIDITY_LSB_DATA        (1)
/****************************************************/
/**\name    ARRAY PARAMETER FOR CALIBRATION     */
/***************************************************/
#define BME280_TEMPERATURE_CALIB_DIG_T1_LSB     (0)
#define BME280_TEMPERATURE_CALIB_DIG_T1_MSB     (1)
#define BME280_TEMPERATURE_CALIB_DIG_T2_LSB     (2)
#define BME280_TEMPERATURE_CALIB_DIG_T2_MSB     (3)
#define BME280_TEMPERATURE_CALIB_DIG_T3_LSB     (4)
#define BME280_TEMPERATURE_CALIB_DIG_T3_MSB     (5)
#define BME280_PRESSURE_CALIB_DIG_P1_LSB       (6)
#define BME280_PRESSURE_CALIB_DIG_P1_MSB       (7)
#define BME280_PRESSURE_CALIB_DIG_P2_LSB       (8)
#define BME280_PRESSURE_CALIB_DIG_P2_MSB       (9)
#define BME280_PRESSURE_CALIB_DIG_P3_LSB       (10)
#define BME280_PRESSURE_CALIB_DIG_P3_MSB       (11)
#define BME280_PRESSURE_CALIB_DIG_P4_LSB       (12)
#define BME280_PRESSURE_CALIB_DIG_P4_MSB       (13)
#define BME280_PRESSURE_CALIB_DIG_P5_LSB       (14)
#define BME280_PRESSURE_CALIB_DIG_P5_MSB       (15)
#define BME280_PRESSURE_CALIB_DIG_P6_LSB       (16)
#define BME280_PRESSURE_CALIB_DIG_P6_MSB       (17)
#define BME280_PRESSURE_CALIB_DIG_P7_LSB       (18)
#define BME280_PRESSURE_CALIB_DIG_P7_MSB       (19)
#define BME280_PRESSURE_CALIB_DIG_P8_LSB       (20)
#define BME280_PRESSURE_CALIB_DIG_P8_MSB       (21)
#define BME280_PRESSURE_CALIB_DIG_P9_LSB       (22)
#define BME280_PRESSURE_CALIB_DIG_P9_MSB       (23)
#define BME280_HUMIDITY_CALIB_DIG_H1           (25)
#define BME280_HUMIDITY_CALIB_DIG_H2_LSB        (0)
#define BME280_HUMIDITY_CALIB_DIG_H2_MSB        (1)
#define BME280_HUMIDITY_CALIB_DIG_H3            (2)
#define BME280_HUMIDITY_CALIB_DIG_H4_MSB        (3)
#define BME280_HUMIDITY_CALIB_DIG_H4_LSB        (4)
#define BME280_HUMIDITY_CALIB_DIG_H5_MSB        (5)
#define BME280_HUMIDITY_CALIB_DIG_H6            (6)
#define BME280_MASK_DIG_H4      (0x0F)
/****************************************************/
/**\name    REGISTER ADDRESS DEFINITIONS  */
/***************************************************/
#define BME280_CHIP_ID_REG                   (0xD0)  /*Chip ID Register */
#define BME280_RST_REG                       (0xE0)  /*Softreset Register */
#define BME280_STAT_REG                      (0xF3)  /*Status Register */
#define BME280_CTRL_MEAS_REG                 (0xF4)  /*Ctrl Measure Register */
#define BME280_CTRL_HUMIDITY_REG             (0xF2)  /*Ctrl Humidity Register*/
#define BME280_CONFIG_REG                    (0xF5)  /*Configuration Register */
#define BME280_PRESSURE_MSB_REG              (0xF7)  /*Pressure MSB Register */
#define BME280_PRESSURE_LSB_REG              (0xF8)  /*Pressure LSB Register */
#define BME280_PRESSURE_XLSB_REG             (0xF9)  /*Pressure XLSB Register */
#define BME280_TEMPERATURE_MSB_REG           (0xFA)  /*Temperature MSB Reg */
#define BME280_TEMPERATURE_LSB_REG           (0xFB)  /*Temperature LSB Reg */
#define BME280_TEMPERATURE_XLSB_REG          (0xFC)  /*Temperature XLSB Reg */
#define BME280_HUMIDITY_MSB_REG              (0xFD)  /*Humidity MSB Reg */
#define BME280_HUMIDITY_LSB_REG              (0xFE)  /*Humidity LSB Reg */
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS  */
/***************************************************/
/* Status Register */
#define BME280_STAT_REG_MEASURING__POS           (3)
#define BME280_STAT_REG_MEASURING__MSK           (0x08)
#define BME280_STAT_REG_MEASURING__LEN           (1)
#define BME280_STAT_REG_MEASURING__REG           (BME280_STAT_REG)

#define BME280_STAT_REG_IM_UPDATE__POS            (0)
#define BME280_STAT_REG_IM_UPDATE__MSK            (0x01)
#define BME280_STAT_REG_IM_UPDATE__LEN            (1)
#define BME280_STAT_REG_IM_UPDATE__REG            (BME280_STAT_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
 FOR TEMPERATURE OVERSAMPLING  */
/***************************************************/
/* Control Measurement Register */
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS             (5)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK             (0xE0)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN             (3)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG             \
(BME280_CTRL_MEAS_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
 FOR PRESSURE OVERSAMPLING  */
/***************************************************/
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS             (2)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK             (0x1C)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN             (3)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG             \
(BME280_CTRL_MEAS_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
 FOR POWER MODE  */
/***************************************************/
#define BME280_CTRL_MEAS_REG_POWER_MODE__POS              (0)
#define BME280_CTRL_MEAS_REG_POWER_MODE__MSK              (0x03)
#define BME280_CTRL_MEAS_REG_POWER_MODE__LEN              (2)
#define BME280_CTRL_MEAS_REG_POWER_MODE__REG              \
(BME280_CTRL_MEAS_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
 FOR HUMIDITY OVERSAMPLING  */
/***************************************************/
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__POS             (0)
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__MSK             (0x07)
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__LEN             (3)
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__REG            \
(BME280_CTRL_HUMIDITY_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
 FOR STANDBY TIME  */
/***************************************************/
/* Configuration Register */
#define BME280_CONFIG_REG_TSB__POS                 (5)
#define BME280_CONFIG_REG_TSB__MSK                 (0xE0)
#define BME280_CONFIG_REG_TSB__LEN                 (3)
#define BME280_CONFIG_REG_TSB__REG                 (BME280_CONFIG_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
 FOR FILTER */
/***************************************************/
#define BME280_CONFIG_REG_FILTER__POS              (2)
#define BME280_CONFIG_REG_FILTER__MSK              (0x1C)
#define BME280_CONFIG_REG_FILTER__LEN              (3)
#define BME280_CONFIG_REG_FILTER__REG              (BME280_CONFIG_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
 FOR PRESSURE AND TEMPERATURE DATA  */
/***************************************************/
/* Data Register */
#define BME280_PRESSURE_XLSB_REG_DATA__POS         (4)
#define BME280_PRESSURE_XLSB_REG_DATA__MSK         (0xF0)
#define BME280_PRESSURE_XLSB_REG_DATA__LEN         (4)
#define BME280_PRESSURE_XLSB_REG_DATA__REG         (BME280_PRESSURE_XLSB_REG)

#define BME280_TEMPERATURE_XLSB_REG_DATA__POS      (4)
#define BME280_TEMPERATURE_XLSB_REG_DATA__MSK      (0xF0)
#define BME280_TEMPERATURE_XLSB_REG_DATA__LEN      (4)
#define BME280_TEMPERATURE_XLSB_REG_DATA__REG      (BME280_TEMPERATURE_XLSB_REG)

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
    int32_t t_fine;

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

} bme280_uncomp_meas;

typedef struct
{
    int fd;
    uint8_t buffer[32];
    bme280_calib_table* p_calib;
    bme280_uncomp_meas* p_uncomp_meas;
    uint8_t power_mode;
    uint8_t filter;
    uint8_t standby_durn;
    uint8_t ovrsmpl_temp;
    uint8_t ovrsmpl_press;
    uint8_t ovrsmpl_humid;
    int32_t comp_temp; // unsigned?
    uint32_t comp_press;
    uint32_t comp_humid;
    uint8_t ctrl_humid_reg;
    uint8_t ctrl_meas_reg;
    uint8_t config_reg;

} bme280;

/**
 * Initialize the bme280.
 * THIS FUNCTION MUST BE CALLED
 * BEFORE ANY OTHER BME280 INTERACTIONS!
 */
uint8_t bme280_init(const char* dev, bme280* inst);
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
uint8_t bme280_powermode(uint8_t mode);
/**
 * Get power mode.
 */
uint8_t bme280_powermode();
/**
 * Set temperature oversample rate.
 */
uint8_t bme280_temp_oversample(uint8_t rate);
/**
 * Get temperature oversample rate.
 */
uint8_t bme280_temp_oversample();
/**
 * Set pressure oversample rate.
 */
uint8_t bme280_press_oversample(uint8_t rate);
/**
 * Get pressure oversample rate.
 */
uint8_t bme280_press_oversample();
/**
 * Set humidity oversample rate.
 */
uint8_t bme280_humid_oversample(uint8_t rate);
/**
 * Get humidity oversample rate.
 */
uint8_t bme280_humid_oversample();
/**
 * Set standby duration.
 */
uint8_t bme280_standby_durn(uint8_t ms);
/**
 * Get standby duration.
 */
uint8_t bme280_standby_durn();
/**
 * Set filter coefficient.
 */
uint8_t bme280_filter(uint8_t coef);
/**
 * Get filter coefficient.
 */
uint8_t bme280_filter();
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

void bme280_delay(uint8_t ms);

#endif /* BME280_IF_SRC_BME280_H_ */
