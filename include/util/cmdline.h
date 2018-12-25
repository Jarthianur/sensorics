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

#pragma once

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

uint16_t parse_port(const char* str, uint16_t def)
{
    uint64_t p = strtoul(str, NULL, 10);
    if (errno == ERANGE || p > UINT16_MAX)
    {
        return def;
    }
    return p;
}

uint32_t parse_interval(const char* str, uint32_t def)
{
    uint64_t t = strtoul(str, NULL, 10);
    if (errno == ERANGE || t > UINT32_MAX)
    {
        return def;
    }
    return t;
}
