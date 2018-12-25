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
#include <unistd.h>

#include <apr.h>
#include <apr_errno.h>
#include <apr_network_io.h>
#include <apr_thread_cond.h>
#include <apr_thread_mutex.h>
#include <apr_thread_proc.h>

#include "util/util.h"

#define DEFAULT_SO_BACKLOG (SOMAXCONN)
#define BUFFSIZE (8129)
#define MAX_CLIENTS (4)
// microsec
#define CONN_TIMEOUT (20000000)

/**
 * Handler for processing clients.
 */
static void* APR_THREAD_FUNC process_client(apr_thread_t* thd, void*);
/**
 * Print error info.
 */
int error(apr_status_t stat);

/**
 * External run status
 */
extern int run_status;
apr_port_t server_port;

int                   clients = 0;
apr_thread_cond_t*    close_cond;
apr_thread_mutex_t*   cond_mutex;
static client_handler handle_client = NULL;

int32_t server_run(int port, client_handler handle, apr_pool_t* parent)
{
    if (handle == NULL)
    {
        return -1;
    }
    else
    {
        handle_client = handle;
    }
    server_port = port;

    apr_socket_t*     so_listen;
    apr_pool_t*       mem_pool;
    apr_status_t      ret_stat;
    apr_threadattr_t* thd_attr;
    apr_sockaddr_t*   sa;

    apr_pool_create(&mem_pool, parent);
    apr_threadattr_create(&thd_attr, mem_pool);
    apr_thread_cond_create(&close_cond, mem_pool);
    apr_thread_mutex_create(&cond_mutex, APR_THREAD_MUTEX_UNNESTED, mem_pool);

    if ((ret_stat = apr_sockaddr_info_get(&sa, NULL, APR_INET, port, 0, mem_pool)) != APR_SUCCESS)
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

    while (run_status == 1)
    {
        apr_socket_t* ns;
        if ((ret_stat = apr_socket_accept(&ns, so_listen, mem_pool)) != APR_SUCCESS)
        {
            error(ret_stat);
            continue;
        }

        if (clients == MAX_CLIENTS)
        {
            apr_socket_shutdown(ns, APR_SHUTDOWN_READWRITE);
            apr_socket_close(ns);
            continue;
        }

        apr_socket_opt_set(ns, APR_SO_NONBLOCK, 0);
        apr_socket_timeout_set(ns, CONN_TIMEOUT);
        apr_thread_t* thd_obj;

        apr_thread_mutex_lock(cond_mutex);
        if ((ret_stat = apr_thread_create(&thd_obj, NULL, process_client, ns, mem_pool)) !=
            APR_SUCCESS)
        {
            error(ret_stat);
        }
        else
        {
            clients++;
        }
        apr_thread_mutex_unlock(cond_mutex);
    }

    apr_thread_mutex_lock(cond_mutex);
    if (clients > 0)
    {
        apr_thread_cond_wait(close_cond, cond_mutex);
    }
    apr_thread_mutex_unlock(cond_mutex);

    apr_thread_cond_destroy(close_cond);
    apr_thread_mutex_destroy(cond_mutex);
    apr_pool_destroy(mem_pool);
    return 0;

err_exit:
{
    char errbuf[256];
    apr_strerror(ret_stat, errbuf, sizeof(errbuf));
    printf("ERROR: %d, %s\n", ret_stat, errbuf);
    apr_thread_cond_destroy(close_cond);
    apr_thread_mutex_destroy(cond_mutex);
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

static void* APR_THREAD_FUNC process_client(_unused_ apr_thread_t* thd, void* data)
{
    apr_socket_t* sock = (apr_socket_t*) data;
    char          buf[BUFFSIZE];

    while (run_status == 1)
    {
        apr_size_t len = handle_client(buf, sizeof(buf));
        apr_socket_send(sock, buf, &len);
        if (len == 0)
        {
            break;
        }
    }
    apr_socket_shutdown(sock, APR_SHUTDOWN_READWRITE);
    apr_socket_close(sock);

    apr_thread_mutex_lock(cond_mutex);
    if (--clients == 0)
    {
        apr_thread_cond_signal(close_cond);
    }
    apr_thread_mutex_unlock(cond_mutex);

    return 0;
}

void server_stop()
{
    run_status = 0;

    apr_sockaddr_t* sa;
    apr_socket_t*   s;
    apr_pool_t*     mem_pool;
    apr_pool_create(&mem_pool, NULL);

    apr_sockaddr_info_get(&sa, "localhost", APR_INET, server_port, 0, mem_pool);
    apr_socket_create(&s, sa->family, SOCK_STREAM, 0, mem_pool);
    apr_socket_connect(s, sa);
    apr_pool_destroy(mem_pool);
    apr_socket_close(s);
}
