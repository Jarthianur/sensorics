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

#include <stddef.h>
#include <stdint.h>

#define _unused_ __attribute__((__unused__))

#define U8_MAX (UINT8_MAX)
#define S8_MAX (INT8_MAX)
#define U8_MIN (0)
#define S8_MIN (INT8_MIN)
#define U16_MAX (UINT16_MAX)
#define S16_MAX (INT16_MAX)
#define U16_MIN (0)
#define S16_MIN (INT16_MIN)
#define U32_MAX (UINT32_MAX)
#define S32_MAX (INT32_MAX)
#define U32_MIN (0)
#define S32_MIN (INT32_MIN)
#define U64_MAX (UINT64_MAX)
#define S64_MAX (INT64_MAX)
#define U64_MIN (0)
#define S64_MIN (INT64_MIN)

#ifndef FALSE
#    define FALSE (0)
#endif
#ifndef TRUE
#    define TRUE (!FALSE)
#endif

typedef uint8_t     u8_t;
typedef int8_t      s8_t;
typedef uint16_t    u16_t;
typedef int16_t     s16_t;
typedef uint32_t    u32_t;
typedef int32_t     s32_t;
typedef uint64_t    u64_t;
typedef int64_t     s64_t;
typedef float       f32_t;
typedef double      f64_t;
typedef int_fast8_t bool_t;
