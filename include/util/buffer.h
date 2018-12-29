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

#include "types.h"

typedef struct
{
    size_t length;
    char*  data;
    size_t allocated;
} buffer;

bool_t BUF_new(buffer* buf, size_t init);
void   BUF_free(buffer* buf);
bool_t BUF_copy(const buffer* src, buffer* dest);
void   BUF_move(buffer* src, buffer* dest);
void   BUF_clear(buffer* buf);
void   BUF_shrink(buffer* buf);
bool_t BUF_append(const buffer* src, buffer* dest);
bool_t BUF_append_cstr(const char* src, buffer* dest);
bool_t BUF_remove(buffer* buf, size_t pos, size_t len);
bool_t BUF_sprintf(buffer* buf, const char* fmt, ...);
