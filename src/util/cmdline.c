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

#include "util/cmdline.h"

#include <errno.h>
#include <stdlib.h>

u16_t CMD_parse_u16(const char* str, u16_t def)
{
    uint64_t p = strtoul(str, NULL, 10);
    if (errno == ERANGE || p > U16_MAX)
    {
        return def;
    }
    return p;
}

u32_t CMD_parse_u32(const char* str, u32_t def)
{
    uint64_t t = strtoul(str, NULL, 10);
    if (errno == ERANGE || t > U32_MAX)
    {
        return def;
    }
    return t;
}
