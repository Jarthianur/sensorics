#include <stdio.h>
#include "bme280.h"

void err(const char* msg)
{
    printf("%s\n", msg);
}

int main(int argc, char** argv)
{
    uint8_t rc = 0;
    // init
    bme280 bme;
    bme280_calib_table ct;
    bme280_uncomp_meas ucm;
    bme.p_calib = &ct;
    bme.p_uncomp_meas = &ucm;
    if (bme280_init("/dev/i2c-1", &bme) == U8_ERROR)
    {
        err("init failed");
        return 1;
    }
    rc = bme280_set_powermode(BME280_NORMAL_MODE);
    rc = bme280_set_humid_oversample(BME280_OVERSAMP_1X);
    rc = bme280_set_temp_oversample(BME280_OVERSAMP_1X);
    rc = bme280_set_press_oversample(BME280_OVERSAMP_1X);
    rc = bme280_set_standby_durn(BME280_STANDBY_TIME_1000_MS);
    rc = bme280_set_filter(BME280_FILTER_COEFF_OFF);
    if (rc != 0)
    {
        err("setup failed");
        return 1;
    }
    bme280_delay(0xFF);
    //test
    rc = bme280_read_burst_tph();
    if (rc != 0)
    {
        err("read failed");
        return 1;
    }

    printf("Temperature in Celsius : %lf C \n", bme280_temp());
    printf("Pressure : %lf hPa \n", bme280_press());
    printf("Relative Humidity : %lf RH \n", bme280_humid());
    //bme280_set_powermode(BME280_SLEEP_MODE);

    //deinit
    if (bme280_deinit() == U8_ERROR)
    {
        err("deinit failed");
        return 1;
    }

    return rc;
}
