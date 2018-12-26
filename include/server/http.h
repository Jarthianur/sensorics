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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "server.h"

static SRV_DEFINE_PROCESS_CLIENT(http_stream)
{
    SRV_CLIENT_INIT
    char buf[8192];
    strcpy(
        buf,
        "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n");
    apr_size_t len = 104;
    apr_socket_send(socket, buf, &len);
    while (server->running)
    {
        len = server->handle_client(buf, sizeof(buf));
        apr_socket_send(socket, buf, &len);
        if (len == 0)
        {
            break;
        }
    }
    strcpy(buf, "0\r\n\r\n");
    len = 6;
    apr_socket_send(socket, buf, &len);
    SRV_CLIENT_DEINIT
}
