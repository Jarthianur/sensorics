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

#include "i2c/i2cif.h"

#include "bme280/bme280.h"
#include "server/server.h"
#include "server/simple_send.h"
#include "util/buffer.h"
#include "util/cmdline.h"
#include "util/logging.h"
#include "util/types.h"

bool_t       handle_client(buffer* buf);
static void* APR_THREAD_FUNC poll_bme280(apr_thread_t* thd, void*);
void                         handle_signal(int signo);

bool_t              run_status = TRUE;
apr_thread_mutex_t* meas_mutex;
f64_t               temperature = 0.0;
f64_t               pressure    = 0.0;
f64_t               humidity    = 0.0;
u32_t               interval    = 1;
size_t              int_count   = 0;
basic_server        server      = {0, FALSE, 0, NULL, NULL, NULL};

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
    BME280_delay(U8_MAX);  // Wait for registers setup time.
    apr_initialize();
    apr_status_t      ret_stat;
    apr_thread_t*     thd_obj;
    apr_pool_t*       mem_pool;
    apr_threadattr_t* thd_attr;
    apr_pool_create(&mem_pool, NULL);
    apr_threadattr_create(&thd_attr, mem_pool);
    apr_thread_mutex_create(&meas_mutex, APR_THREAD_MUTEX_UNNESTED, mem_pool);

    if ((ret_stat = apr_thread_create(&thd_obj, NULL, poll_bme280, &bme, mem_pool)) != APR_SUCCESS)
    {
        LOG("create thread failed");
    }
    apr_signal(SIGINT, handle_signal);
    apr_signal(SIGKILL, handle_signal);
    apr_signal(SIGPIPE, SIG_IGN);
    server.port          = port;
    server.handle_client = handle_client;
    SRV_run(&server, simple_send, mem_pool);
    // shutdown
    apr_thread_join(&ret_stat, thd_obj);
    apr_thread_mutex_destroy(meas_mutex);
    apr_pool_destroy(mem_pool);
    apr_terminate();
    BME280_set_powermode(&bme, BME280_SLEEP_MODE);
    BME280_deinit(&bme);

    return rc;
}

u8_t checksum(const buffer* buf)
{
    u8_t csum = 0;
    for (size_t i = 1; i < buf->length - 2; ++i)  // ignore $,* and 0-byte
    {
        csum ^= (u8_t) buf->data[i];
    }

    return csum;
}

bool_t handle_client(buffer* buf)
{
    sleep(interval);
    apr_thread_mutex_lock(meas_mutex);
    if (!BUF_sprintf(buf, "$WIMDA,%.2lf,I,%.4lf,B,%.1lf,C,,,%.1lf,,,,,,,,,,,*", pressure * 0.02953,
                     pressure / 1000.0, temperature, humidity))
    {
        return FALSE;
    }
    apr_thread_mutex_unlock(meas_mutex);
    u8_t csum = checksum(buf);
    char end[8];
    if (snprintf(end, sizeof(end), "%02x\r\n", csum) < 0)
    {
        return FALSE;
    }
    if (!BUF_append_cstr(end, buf))
    {
        return FALSE;
    }
    return TRUE;
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
