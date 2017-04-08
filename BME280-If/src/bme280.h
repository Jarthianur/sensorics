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
#define BME280_REG_CTRLHUMID     (0xF2)
#define BME280_REG_CTRL          (0xF4)
#define BME280_REG_CONF          (0xF5)
#define BME280_REG_PRESS         (0xF7)
#define BME280_REG_TEMP          (0xFA)
#define BME280_REG_HUMID         (0xFD)
#define BME280_REG_MEAS          (BME280_REG_PRESS)

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

int32_t unpack_press();
int32_t unpack_temp();
int32_t unpack_humid();

uint8_t check_null();

#endif /* BME280_IF_SRC_BME280_H_ */
