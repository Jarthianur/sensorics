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

#define LOG_COMPONENT "main"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <apr_errno.h>
#include <apr_signal.h>

#include "bme280/bme280.h"
#include "server/server.h"
#include "server/simple_send.h"
#include "sql/sqlite.h"
#include "util/cmdline.h"
#include "util/logging.h"
#include "util/types.h"

/**
 * Produce WIMDA sentence and store into buff.
 */
size_t handle_client(char* buff, size_t len);

/**
 * Handler to poll temp, press, humid from bme280 sensor.
 */
static void* APR_THREAD_FUNC poll_bme280(apr_thread_t* thd, void*);

/**
 * Exit signal handler.
 */
void handle_signal(int signo);

/**
 * Global run status.
 */
bool_t run_status = TRUE;

/**
 * Mutex to gain threadsafety for temp, press, humid data.
 */
apr_thread_mutex_t* meas_mutex;

f64_t temperature = 0.0;
f64_t pressure    = 0.0;
f64_t humidity    = 0.0;

u32_t  interval  = 1;
size_t int_count = 0;

basic_server server = {0, FALSE, 0, NULL, NULL, NULL};

sql_db db;

int main(int argc, char** argv)
{
    u16_t port = SRV_DEFAULT_PORT;
    s32_t arg;
    for (arg = 1; arg < argc; ++arg)
    {
        if (strcmp(argv[arg], "-p") == 0 && arg < argc - 1)
        {
            port = CMD_parse_u16(argv[arg + 1], SRV_DEFAULT_PORT);
        }
        else if (strcmp(argv[arg], "-t") == 0 && arg < argc - 1)
        {
            interval = CMD_parse_u32(argv[arg + 1], 1);
        }
    }
    LOGF("Using interval: %u", interval);
    LOGF("Using port: %hu", port);

    // Initialize BME280
    u8_t   rc = 0;
    bme280 bme;

    if (BME280_init("/dev/i2c-1", &bme) == BME280_ERROR)
    {
        LOG("init failed");
        return 1;
    }
    rc |= BME280_set_powermode(&bme, BME280_NORMAL_MODE);
    rc |= BME280_set_humid_oversample(&bme, BME280_OVERSAMP_1X);
    rc |= BME280_set_temp_oversample(&bme, BME280_OVERSAMP_1X);
    rc |= BME280_set_press_oversample(&bme, BME280_OVERSAMP_1X);
    rc |= BME280_set_standby_durn(&bme, BME280_STANDBY_TIME_500_MS);
    rc |= BME280_set_filter(&bme, BME280_FILTER_COEFF_OFF);

    if (rc != 0)
    {
        LOG("setup failed");
    }
    // Wait for registers setup time.
    BME280_delay(U8_MAX);

    // Initialize routine
    apr_initialize();

    apr_status_t      ret_stat;
    apr_thread_t*     thd_obj;
    apr_pool_t*       mem_pool;
    apr_threadattr_t* thd_attr;

    apr_pool_create(&mem_pool, NULL);
    apr_threadattr_create(&thd_attr, mem_pool);
    apr_thread_mutex_create(&meas_mutex, APR_THREAD_MUTEX_UNNESTED, mem_pool);

    // Spawn poll thread
    if ((ret_stat = apr_thread_create(&thd_obj, NULL, poll_bme280, &bme, mem_pool)) != APR_SUCCESS)
    {
        LOG("create thread failed");
    }

    // register signal handlers
    apr_signal(SIGINT, handle_signal);
    apr_signal(SIGKILL, handle_signal);
    apr_signal(SIGPIPE, SIG_IGN);

    db.db_file = "sensor.db";
    if (!SQL_open(&db))
    {
        LOG("Could not open database");
    }
    sql_stmt stmt = {
        "DROP TABLE IF EXISTS sensor;CREATE TABLE sensor(time TEXT, temp REAL, press REAL, humid REAL);",
        NULL};
    SQL_exec(&db, &stmt);

    // Run server
    server.port          = port;
    server.handle_client = handle_client;
    SRV_run(&server, simple_send, mem_pool);

    // De-initialize
    apr_thread_join(&ret_stat, thd_obj);
    apr_thread_mutex_destroy(meas_mutex);
    apr_pool_destroy(mem_pool);
    apr_terminate();
    BME280_set_powermode(&bme, BME280_SLEEP_MODE);

    BME280_deinit(&bme);
    SQL_close(&db);
    return rc;
}

/**
 * Compute checksum for NMEA sentence.
 */
u8_t checksum(const char* sentence, size_t size)
{
    u8_t   csum = 0;
    size_t i    = 1;  // $ in nmea str not included
    while (sentence[i] != '*' && sentence[i] != '\0' && i < size)
    {
        csum ^= (u8_t) sentence[i++];
    }
    return csum;
}

size_t handle_client(char* buf, size_t len)
{
    sleep(interval);
    s32_t rc = 0;
    apr_thread_mutex_lock(meas_mutex);
    if (int_count++ % 600 == 0)
    {
        int_count = 0;
        sql_stmt stmt;
        SQL_prepare(&stmt, "INSERT INTO sensor VALUES(CURRENT_TIME,%lf,%lf,%lf);", temperature,
                    pressure / 1000.0, humidity);
        SQL_exec(&db, &stmt);
        free(stmt.query);
    }
    if ((rc = snprintf(buf, len, "$WIMDA,%.2lf,I,%.4lf,B,%.1lf,C,,,%.1lf,,,,,,,,,,,*",
                       pressure * 0.02953, pressure / 1000.0, temperature, humidity)) < 0)
    {
        return 0;
    }
    apr_thread_mutex_unlock(meas_mutex);
    u8_t   csum = checksum(buf, rc);
    char   end[8];
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

static void* APR_THREAD_FUNC poll_bme280(_unused_ apr_thread_t* thd, void* data)
{
    bme280* bme = (bme280*) data;
    while (run_status)
    {
        apr_thread_mutex_lock(meas_mutex);
        if (BME280_read_burst_tph(bme) == BME280_ERROR)
        {
            LOG("read from i2c failed");
            apr_thread_mutex_unlock(meas_mutex);
            break;
        }
        temperature = BME280_temp(bme);
        pressure    = BME280_press(bme);
        humidity    = BME280_humid(bme);
        apr_thread_mutex_unlock(meas_mutex);
        sleep(interval);
    }
    return 0;
}

void handle_signal(int signo)
{
    LOGF("caught signal: %d", signo);
    run_status = FALSE;
    SRV_stop(&server);
}
