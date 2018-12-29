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

#include "server/server.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <apr_errno.h>

#define DEFAULT_SO_BACKLOG (SOMAXCONN)
#define MAX_CLIENTS (4)
// microsec
#define CONN_TIMEOUT (20000000)

/**
 * Print error info.
 */
int error(apr_status_t stat);

s32_t SRV_run(basic_server* server, SRV_thread_handle thd_handle, apr_pool_t* parent)
{
    if (server->handle_client == NULL)
    {
        return -1;
    }

    apr_socket_t*     so_listen;
    apr_pool_t*       mem_pool;
    apr_status_t      ret_stat;
    apr_threadattr_t* thd_attr;
    apr_sockaddr_t*   sa;

    apr_pool_create(&mem_pool, parent);
    apr_threadattr_create(&thd_attr, mem_pool);
    apr_thread_cond_create(&server->close_cond, mem_pool);
    apr_thread_mutex_create(&server->cond_mutex, APR_THREAD_MUTEX_UNNESTED, mem_pool);

    if ((ret_stat = apr_sockaddr_info_get(&sa, NULL, APR_INET, server->port, 0, mem_pool)) !=
        APR_SUCCESS)
    {
        goto err_exit;
    }

    if ((ret_stat = apr_socket_create(&so_listen, sa->family, SOCK_STREAM, APR_PROTO_TCP,
                                      mem_pool)) != APR_SUCCESS)
    {
        goto err_exit;
    }

    apr_socket_opt_set(so_listen, APR_SO_NONBLOCK, 0);
    apr_socket_timeout_set(so_listen, -1);
    apr_socket_opt_set(so_listen, APR_SO_REUSEADDR, 1);

    if ((ret_stat = apr_socket_bind(so_listen, sa)) != APR_SUCCESS)
    {
        goto err_exit;
    }

    if ((ret_stat = apr_socket_listen(so_listen, DEFAULT_SO_BACKLOG)) != APR_SUCCESS)
    {
        goto err_exit;
    }

    server->running = TRUE;

    while (server->running)
    {
        apr_socket_t* socket;
        if ((ret_stat = apr_socket_accept(&socket, so_listen, mem_pool)) != APR_SUCCESS)
        {
            error(ret_stat);
            continue;
        }

        if (server->clients == MAX_CLIENTS)
        {
            apr_socket_shutdown(socket, APR_SHUTDOWN_READWRITE);
            apr_socket_close(socket);
            continue;
        }

        apr_socket_opt_set(socket, APR_SO_NONBLOCK, 0);
        apr_socket_timeout_set(socket, CONN_TIMEOUT);
        apr_thread_t* thd_obj;

        apr_thread_mutex_lock(server->cond_mutex);
        client_data* data = NULL;
        if ((data = malloc(sizeof(client_data))) == NULL)
        {
            continue;
        }
        data->server = server;
        data->socket = socket;
        if ((ret_stat = apr_thread_create(&thd_obj, NULL, thd_handle, data, mem_pool)) !=
            APR_SUCCESS)
        {
            error(ret_stat);
        }
        else
        {
            ++(server->clients);
        }
        apr_thread_mutex_unlock(server->cond_mutex);
    }

    apr_thread_mutex_lock(server->cond_mutex);
    if (server->clients > 0)
    {
        apr_thread_cond_wait(server->close_cond, server->cond_mutex);
    }
    apr_thread_mutex_unlock(server->cond_mutex);

    apr_thread_cond_destroy(server->close_cond);
    apr_thread_mutex_destroy(server->cond_mutex);
    apr_pool_destroy(mem_pool);
    return 0;

err_exit:
{
    char errbuf[256];
    apr_strerror(ret_stat, errbuf, sizeof(errbuf));
    printf("ERROR: %d, %s\n", ret_stat, errbuf);
    apr_thread_cond_destroy(server->close_cond);
    apr_thread_mutex_destroy(server->cond_mutex);
    apr_pool_destroy(mem_pool);
    return -1;
}
}

int error(apr_status_t stat)
{
    char errbuf[256];
    apr_strerror(stat, errbuf, sizeof(errbuf));
    printf("ERROR: %d, %s\n", stat, errbuf);
    return -1;
}

void SRV_stop(basic_server* server)
{
    server->running = FALSE;

    apr_sockaddr_t* sa;
    apr_socket_t*   s;
    apr_pool_t*     mem_pool;
    apr_pool_create(&mem_pool, NULL);

    apr_sockaddr_info_get(&sa, "localhost", APR_INET, server->port, 0, mem_pool);
    apr_socket_create(&s, sa->family, SOCK_STREAM, 0, mem_pool);
    apr_socket_connect(s, sa);
    apr_pool_destroy(mem_pool);
    apr_socket_close(s);
}
