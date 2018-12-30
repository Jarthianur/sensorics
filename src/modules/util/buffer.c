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

#include "util/buffer.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool_t BUF_maybe_realloc(buffer* buf, size_t len);

bool_t BUF_new(buffer* buf, size_t init)
{
    if ((buf->data = calloc(init, sizeof(char))) == NULL)
    {
        return FALSE;
    }
    buf->length    = 0;
    buf->allocated = init;
    return TRUE;
}

void BUF_free(buffer* buf)
{
    free(buf->data);
    buf->data      = NULL;
    buf->length    = 0;
    buf->allocated = 0;
}

bool_t BUF_copy(const buffer* src, buffer* dest)
{
    if (!BUF_maybe_realloc(dest, src->length))
    {
        return FALSE;
    }
    strncpy(dest->data, src->data, dest->allocated);
    dest->length                                         = src->length;
    dest->data[dest->length == 0 ? 0 : dest->length - 1] = 0;
    return TRUE;
}

void BUF_move(buffer* src, buffer* dest)
{
    if (dest->data != NULL)
    {
        free(dest->data);
    }
    dest->data      = src->data;
    dest->length    = src->length;
    dest->allocated = src->allocated;
    src->data       = NULL;
    src->length     = 0;
    src->allocated  = 0;
}

void BUF_clear(buffer* buf)
{
    buf->length = 0;
    if (buf->allocated > 0)
    {
        buf->data[0] = 0;
    }
}

void BUF_shrink(buffer* buf)
{
    char* new_data = realloc(buf->data, buf->length);
    if (new_data != NULL)
    {
        buf->data = new_data;
    }
}

bool_t BUF_append(const buffer* src, buffer* dest)
{
    size_t len = dest->length + src->length - 1;  // only 1 null byte
    if (!BUF_maybe_realloc(dest, len))
    {
        return FALSE;
    }
    strncat(dest->data, src->data, dest->allocated);
    return TRUE;
}

bool_t BUF_append_cstr(const char* src, buffer* dest)
{
    size_t len = dest->length + strlen(src);
    if (!BUF_maybe_realloc(dest, len))
    {
        return FALSE;
    }
    strncat(dest->data, src, dest->allocated);
    return TRUE;
}

bool_t BUF_remove(buffer* buf, size_t pos, size_t len)
{
    if (pos >= buf->length - 1)
    {
        return FALSE;
    }
    if (len == 0 || pos + len >= buf->length)
    {
        buf->data[pos] = 0;
        buf->length    = pos + 1;
        return TRUE;
    }
    memmove(buf->data + pos, buf->data + pos + len, buf->length - pos - len);
    buf->length -= len;
    buf->data[buf->length - 1] = 0;  // just to be sure
    return TRUE;
}

bool_t BUF_sprintf(buffer* buf, const char* fmt, ...)
{
    va_list args, args_cp;
    va_start(args, fmt);
    va_copy(args_cp, args);
    s32_t len = vsnprintf(NULL, 0, fmt, args) + 1;
    va_end(args);
    if (len < 0 && !BUF_maybe_realloc(buf, len))
    {
        return FALSE;
    }
    vsnprintf(buf->data, buf->allocated, fmt, args_cp);
    va_end(args_cp);
    return TRUE;
}

bool_t BUF_maybe_realloc(buffer* buf, size_t len)
{
    if (buf->data == NULL || buf->allocated < len)
    {
        char* new_data = realloc(buf->data, len);
        if (new_data == NULL)
        {
            return FALSE;
        }
        buf->data      = new_data;
        buf->allocated = len;
    }
    return TRUE;
}
