#include <apr-1.0/apr.h>
#include <apr-1.0/apr_errno.h>
#include <apr-1.0/apr_general.h>
#include <apr-1.0/apr_pools.h>
#include <apr-1.0/apr_thread_mutex.h>
#include <apr-1.0/apr_thread_proc.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "../server/server.h"
#include "bme280.h"

void err(const char* msg)
{
    printf("%s\n", msg);
}

int handle(char* buf, size_t len);
static void* APR_THREAD_FUNC poll_bme280(apr_thread_t *thd, void *);

apr_thread_mutex_t* mutex;
double temperature = 0.0;
double pressure = 0.0;
double humidity = 0.0;

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

    // begin work
    apr_initialize();

    apr_status_t ret_stat;
    apr_thread_t *thd_obj;
    apr_pool_t *mem_pool;
    apr_threadattr_t *thd_attr;

    apr_pool_create(&mem_pool, NULL);
    apr_threadattr_create(&thd_attr, mem_pool);
    apr_thread_mutex_create(&mutex, APR_THREAD_MUTEX_UNNESTED, mem_pool);

    if ((ret_stat = apr_thread_create(&thd_obj, NULL, poll_bme280, 0, mem_pool)) != APR_SUCCESS)
    {
        printf("Error Creating new Thread\n");
    }

    server_run(SERVER_DEFAULT_PORT, handle, mem_pool);

    apr_thread_join(&ret_stat, thd_obj);
    apr_pool_destroy(mem_pool);
    apr_terminate();

    bme280_set_powermode(BME280_SLEEP_MODE);

    //deinit
    if (bme280_deinit() == U8_ERROR)
    {
        err("deinit failed");
        return 1;
    }

    return rc;
}

int32_t checksum(const char* sentence, size_t size)
{
    int32_t csum = 0;
    size_t i = 1; // $ in nmea str not included
    while (sentence[i] != '*' && sentence[i] != '\0' && i < size)
    {
        csum ^= (int32_t) sentence[i++];
    }
    return csum;
}

int handle(char* buf, size_t len)
{
    int32_t rc = 0;
    apr_thread_mutex_lock(mutex);
    rc = snprintf(buf, len, "$WIMDA,%.2lf,I,%.3lf,B,%.1lf,C,,,%.1lf,,,,,,,,,,,*",
                  pressure * 0.02953, pressure, temperature, humidity);
    apr_thread_mutex_unlock(mutex);
    int32_t csum = checksum(buf, rc);
    char end[8];
    rc += snprintf(end, 8, "%02x\r\n", csum);
    if (rc < len)
    {
        strcat(buf, end);
    }
    return rc;
}

static void* APR_THREAD_FUNC poll_bme280(apr_thread_t *thd, void * data)
{
    while (1)
    {
        apr_thread_mutex_lock(mutex);
        if (bme280_read_burst_tph() == U8_ERROR)
        {
            apr_thread_mutex_unlock(mutex);
            break;
        }
        temperature = bme280_temp();
        pressure = bme280_press();
        humidity = bme280_humid();
        apr_thread_mutex_unlock(mutex);
        sleep(1);
    }
    return (void*) apr_thread_exit(thd, 0);
}
