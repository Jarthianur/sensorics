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

#include <sqlite3.h>

#include "util/types.h"

typedef struct
{
    const char* db_file;
    sqlite3*    db;
} sql_db;

typedef struct
{
    size_t size;
    void*  data;
    bool_t valid;
} sql_result;

typedef struct
{
    char*         query;
    sqlite3_stmt* stmt;
} sql_stmt;

bool_t SQL_open(sql_db* db);
void   SQL_close(sql_db* db);
sql_result SQL_exec(sql_db* db, sql_stmt* stmt);
bool_t     SQL_prepare(sql_stmt* stmt, const char* fmt, ...);
