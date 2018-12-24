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

#include <apr-1/apr.h>
#include <apr-1/apr_errno.h>
#include <apr-1/apr_general.h>
#include <apr-1/apr_pools.h>
#include <apr-1/apr_signal.h>
#include <apr-1/apr_thread_mutex.h>
#include <apr-1/apr_thread_proc.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "../server/server.h"
#include "../util/util.h"
#include "bme280.h"

/**
 * Produce WIMDA sentence and store into buff.
 */
size_t handle(char* buff, size_t len);
/**
 * Handler to poll temp, press, humid from bme280 sensor.
 */
static void* APR_THREAD_FUNC poll_bme280(apr_thread_t *thd, void *);
/**
 * Exit signal handler.
 */
void handle_signal(int signo);

/**
 * Global run status.
 * 1 = run
 * 0 = stop
 */
int run_status = 1;

/**
 * Mutex to gain threadsafety for temp, press, humid data.
 */
apr_thread_mutex_t* meas_mutex;

double temperature = 0.0;
double pressure = 0.0;
double humidity = 0.0;


uint32_t interval = 1;

int main(int argc, char** argv)
{
    uint16_t port = 0;
    int32_t arg;
    for (arg = 1; arg < argc; ++arg)
    {
        if (strcmp(argv[arg],"-p") == 0 && arg < argc - 1)
        {
            port = parse_port(argv[arg+1], SERVER_DEFAULT_PORT);
        }
        else if (strcmp(argv[arg],"-t") == 0 && arg < argc - 1)
        {
            interval = parse_interval(argv[arg+1], 1);
        }
    }

    // Initialize BME280
    uint8_t rc = 0;
    bme280 bme;
    bme280_calib_table ct;
    bme280_uncomp_meas ucm;
    bme.p_calib = &ct;
    bme.p_uncomp_meas = &ucm;

    if (bme280_init("/dev/i2c-1", &bme) == U8_ERROR)
    {
        printf("init failed\n");
        return 1;
    }
    rc = bme280_set_powermode(BME280_NORMAL_MODE);
    rc = bme280_set_humid_oversample(BME280_OVERSAMP_1X);
    rc = bme280_set_temp_oversample(BME280_OVERSAMP_1X);
    rc = bme280_set_press_oversample(BME280_OVERSAMP_1X);
    rc = bme280_set_standby_durn(BME280_STANDBY_TIME_500_MS);
    rc = bme280_set_filter(BME280_FILTER_COEFF_OFF);

    if (rc != 0)
    {
        printf("setup failed\n");
        return 1;
    }
    // Wait for registers setup time.
    bme280_delay(0xFF);

    // Initialize routine
    apr_initialize();

    apr_status_t ret_stat;
    apr_thread_t *thd_obj;
    apr_pool_t *mem_pool;
    apr_threadattr_t *thd_attr;

    apr_pool_create(&mem_pool, NULL);
    apr_threadattr_create(&thd_attr, mem_pool);
    apr_thread_mutex_create(&meas_mutex, APR_THREAD_MUTEX_UNNESTED, mem_pool);

    // Spawn poll thread
    if ((ret_stat = apr_thread_create(&thd_obj, NULL, poll_bme280, &interval, mem_pool)) != APR_SUCCESS)
    {
        printf("create thread failed\n");
    }

    // register signal handlers
    apr_signal(SIGINT, handle_signal);
    apr_signal(SIGKILL, handle_signal);
    apr_signal(SIGPIPE, SIG_IGN);

    // Run server
    server_run(port, handle, mem_pool);

    // De-initialize
    apr_thread_join(&ret_stat, thd_obj);
    apr_thread_mutex_destroy(meas_mutex);
    apr_pool_destroy(mem_pool);
    apr_terminate();
    bme280_set_powermode(BME280_SLEEP_MODE);

    bme280_deinit();
    return rc;
}

/**
 * Compute checksum for NMEA sentence.
 */
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

size_t handle(char* buf, size_t len)
{
    sleep(interval);
    int32_t rc = 0;
    apr_thread_mutex_lock(meas_mutex);
    if ((rc = snprintf(buf, len, "$WIMDA,%.2lf,I,%.4lf,B,%.1lf,C,,,%.1lf,,,,,,,,,,,*",
                  pressure * 0.02953, pressure / 1000.0, temperature, humidity)) < 0)
    {
        return 0;
    }
    apr_thread_mutex_unlock(meas_mutex);
    int32_t csum = checksum(buf, rc);
    char end[8];
    size_t l = rc;
    if ((rc = snprintf(end, 8, "%02x\r\n", csum)) < 0)
    {
        return 0;
    }
    l += rc;
    if (l < len)
    {
        strcat(buf, end);
        return l;
    }
    return 0;
}

static void* APR_THREAD_FUNC poll_bme280(apr_thread_t *thd, void * data)
{
    while (run_status == 1)
    {
        apr_thread_mutex_lock(meas_mutex);
        if (bme280_read_burst_tph() == U8_ERROR)
        {
            printf("read from i2c failed\n");
            apr_thread_mutex_unlock(meas_mutex);
            break;
        }
        temperature = bme280_temp();
        pressure = bme280_press();
        humidity = bme280_humid();
        apr_thread_mutex_unlock(meas_mutex);
        sleep(interval);
    }
    return 0;
}

void handle_signal(int signo)
{
    printf("caught signal: %d\n", signo);
    run_status = 0;
    server_stop();
}
