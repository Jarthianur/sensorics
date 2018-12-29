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

#include <stdlib.h>
#include <string.h>

bool_t BUF_new(buffer* buf, size_t init)
{
    if (buf->data != NULL)
    {
        return FALSE;
    }
    if ((buf->data = malloc(sizeof(char) * init)) == NULL)
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
    buf->length    = 0;
    buf->allocated = 0;
}

bool_t BUF_copy(const buffer* src, buffer* dest)
{
    if (src->data == NULL)
    {
        dest->length = 0;
        return TRUE;
    }
    if (dest->data == NULL || dest->allocated < src->length)
    {
        char* new_data = realloc(dest->data, src->length);
        if (new_data == NULL)
        {
            return FALSE;
        }
        dest->data      = new_data;
        dest->allocated = src->length;
    }
    strncpy(dest->data, src->data, dest->allocated);
    dest->length = src->length;
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
}

void BUF_shrink(buffer* buf)
{
    char* new_data = realloc(buf->data, buf->length);
    if (new_data != NULL)
    {
        buf->data = new_data;
    }
}

bool_t BUF_append(const buffer* src, buffer* dest) {}

bool_t BUF_append_cstr(const char* src, buffer* dest) {}

bool_t BUF_remove(buffer* buf, size_t pos, size_t len) {}

bool_t BUF_sprintf(buffer* buf, const char* fmt, ...) {}
