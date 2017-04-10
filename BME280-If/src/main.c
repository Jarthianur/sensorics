#include <stdio.h>
#include "bme280.h"

void err(const char* msg)
{
    printf("%s\n", msg);
}

int main(int argc, char** argv)
{
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
    bme280_powermode(BME280_NORMAL_MODE);
    bme280_humid_oversample(BME280_OVERSAMP_1X);
    bme280_temp_oversample(BME280_OVERSAMP_1X);
    bme280_press_oversample(BME280_OVERSAMP_1X);

    //test
    bme280_read_burst_tph();

    printf("Temperature in Celsius : %.2lf C \n", bme280_temp());
    printf("Pressure : %.2lf hPa \n", bme280_press());
    printf("Relative Humidity : %.2lf RH \n", bme280_humid());

    //deinit
    bme280_deinit();

    return 0;
}
