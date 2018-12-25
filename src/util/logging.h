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

#ifndef SRC_UTIL_LOGGING_H_
#define SRC_UTIL_LOGGING_H_

#include <stdio.h>

#ifndef LOG_COMPONENT
#define LOG_COMPONENT ""
#endif

#define LOG(MSG) printf(LOG_COMPONENT "%s\n", MSG)
#define LOGF(FMT, ...) printf(LOG_COMPONENT FMT, __VA_ARGS__)

#endif // SRC_UTIL_LOGGING_H_
