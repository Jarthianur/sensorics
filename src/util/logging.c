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

#include "util/logging.h"

#include <time.h>

char LOG_time_str[32] = "";

const char* LOG_get_time()
{
    time_t     t;
    struct tm* ti;
    time(&t);
    ti = localtime(&t);
    snprintf(LOG_time_str, sizeof(LOG_time_str), "%s", asctime(ti));
    return LOG_time_str;
}
