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
#include <stdlib.h>
#include <string.h>

#include <apr.h>
#include <apr_general.h>
#include <apr_signal.h>

#include "i2c/i2cif.h"

#include "bme280/bme280.h"
#include "sql/sqlite.h"
#include "util/buffer.h"
#include "util/cmdline.h"
#include "util/logging.h"
#include "util/types.h"

void handle_signal(int signo);

bool_t run_status = TRUE;

int main(int argc, char** argv)
{
    f64_t  temperature = 0.0;
    f64_t  pressure    = 0.0;
    f64_t  humidity    = 0.0;
    u32_t  interval    = 1;
    sql_db db;
    db.db_file = "/tmp/sensor.db";

    for (s32_t arg = 1; arg < argc; ++arg)
    {
        if (strcmp(argv[arg], "-t") == 0 && arg < argc - 1)
        {
            interval = CMD_parse_u32(argv[arg + 1], 1);
        }
        else if (strcmp(argv[arg], "-d") == 0 && arg < argc - 1)
        {
            db.db_file = argv[arg + 1];
        }
    }
    LOGF("Using interval: %u", interval);
    LOGF("Using database: %s", db.db_file);
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
    apr_signal(SIGINT, handle_signal);
    apr_signal(SIGKILL, handle_signal);
    apr_signal(SIGPIPE, SIG_IGN);

    if (!SQL_open(&db))
    {
        LOG("Could not open database");
    }
    sql_stmt stmt = {
        "DROP TABLE IF EXISTS sensor;CREATE TABLE sensor(time TEXT, temp REAL, press REAL, humid REAL);"
        /*, NULL*/};

    if (!SQL_exec(&db, &stmt).valid)
    {
        LOG("Table creation failed");
    }
    while (run_status)
    {
        if (BME280_read_burst_tph(&bme) == BME280_ERROR)
        {
            LOG("read from i2c failed");
            break;
        }
        temperature = BME280_temp(&bme);
        pressure    = BME280_press(&bme);
        humidity    = BME280_humid(&bme);
        sql_stmt stmt;
        SQL_prepare(&stmt, "INSERT INTO sensor VALUES(CURRENT_TIME,%.1lf,%.4lf,%.1lf);",
                    temperature, pressure, humidity);

        if (!SQL_exec(&db, &stmt).valid)
        {
            LOG("Export to database failed");
        }
        SQL_finalize(&stmt);
        u32_t int_count = 0;

        while (int_count++ < interval && run_status)
        {
            sleep(1);
        }
    }
    apr_terminate();
    BME280_set_powermode(&bme, BME280_SLEEP_MODE);
    BME280_deinit(&bme);
    SQL_close(&db);

    return rc;
}

void handle_signal(int signo)
{
    LOGF("caught signal: %d", signo);
    run_status = FALSE;
}
