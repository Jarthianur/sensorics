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

#include "sql/sqlite.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "util/buffer.h"

bool_t SQL_open(sql_db* db)
{
    int rc = sqlite3_open(db->db_file, &(db->db));
    if (rc != SQLITE_OK)
    {
        sqlite3_close(db->db);
        return FALSE;
    }
    return TRUE;
}

void SQL_close(sql_db* db)
{
    sqlite3_close(db->db);
}

sql_result SQL_exec(sql_db* db, sql_stmt* stmt)
{
    sql_result res = {0, NULL, FALSE};
    int        rc  = sqlite3_exec(db->db, stmt->query, NULL, NULL, NULL);
    if (rc == SQLITE_OK)
    {
        res.valid = TRUE;
    }
    return res;
}

bool_t SQL_prepare(sql_stmt* stmt, const char* fmt, ...)
{
    buffer buf;
    BUF_new(&buf, 128);
    va_list args;
    va_start(args, fmt);
    bool_t res = BUF_vsprintf(&buf, fmt, args);
    va_end(args);
    BUF_shrink(&buf);
    stmt->query = buf.data;
    return res;
}

void SQL_finalize(sql_stmt* stmt)
{
    free(stmt->query);
}
