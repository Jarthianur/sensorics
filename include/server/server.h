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

#include <apr.h>
#include <apr_general.h>
#include <apr_network_io.h>
#include <apr_pools.h>
#include <apr_thread_cond.h>
#include <apr_thread_mutex.h>
#include <apr_thread_proc.h>

#include "util/buffer.h"
#include "util/types.h"

#define SRV_DEFAULT_PORT (7997)

#define SRV_CLIENT_INIT                           \
    apr_socket_t* socket;                         \
    basic_server* server;                         \
    {                                             \
        client_data* cdata = (client_data*) data; \
        socket             = cdata->socket;       \
        server             = cdata->server;       \
        free(data);                               \
        data = NULL;                              \
    }

#define SRV_CLIENT_DEINIT                                \
    apr_socket_shutdown(socket, APR_SHUTDOWN_READWRITE); \
    apr_socket_close(socket);                            \
    apr_thread_mutex_lock(server->cond_mutex);           \
    if (--(server->clients) == 0)                        \
    {                                                    \
        apr_thread_cond_signal(server->close_cond);      \
    }                                                    \
    apr_thread_mutex_unlock(server->cond_mutex);         \
    return NULL;

#define SRV_DEFINE_PROCESS_CLIENT(M) void* APR_THREAD_FUNC M(_unused_ apr_thread_t* thd, void* data)

/**
 * Prototype of a client handler.
 * Such a function takes a char-ptr
 * and the respective array size. The result must
 * be stored in the char-ptr and will be sent to the client.
 * @param char* target array to store data
 * @param size_t array length
 * @return chars (bytes) written to the ptr
 */
typedef bool_t (*SRV_client_handler)(buffer* /*buf*/);

typedef void* APR_THREAD_FUNC (*SRV_thread_handle)(apr_thread_t*, void*);

typedef struct
{
    size_t              clients;
    bool_t              running;
    apr_port_t          port;
    apr_thread_cond_t*  close_cond;
    apr_thread_mutex_t* cond_mutex;
    SRV_client_handler  handle_client;
} basic_server;

typedef struct
{
    basic_server* server;
    apr_socket_t* socket;
} client_data;

bool_t SRV_run(basic_server* server, SRV_thread_handle thd_handle, apr_pool_t* parent);

void SRV_stop(basic_server* server);
