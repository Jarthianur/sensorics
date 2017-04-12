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

#include "server.h"
#include <apr-1.0/apr.h>
#include <apr-1.0/apr_errno.h>
#include <apr-1.0/apr_network_io.h>
#include <apr-1.0/apr_thread_proc.h>
#include <stdio.h>
#include <unistd.h>

#define DEFAULT_SO_BACKLOG  (SOMAXCONN)
#define BUFFSIZE            (8129)
#define SYNC_TIME           (1)
#define MAX_CLIENTS         (4)

/**
 * Handler for processing clients.
 */
static void* APR_THREAD_FUNC process_client(apr_thread_t *thd, void *);
/**
 * Print error info.
 */
int error(apr_status_t stat);

/**
 * External run status
 */
extern int run_status;
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

    apr_socket_t *so_listen;
    apr_pool_t *mem_pool;
    apr_status_t ret_stat;
    apr_threadattr_t *thd_attr;
    apr_sockaddr_t *sa;

    apr_pool_create(&mem_pool, parent);
    apr_threadattr_create(&thd_attr, mem_pool);

    if ((ret_stat = apr_sockaddr_info_get(&sa, NULL, APR_INET, port, 0, mem_pool)) != APR_SUCCESS)
    {
        apr_pool_destroy(mem_pool);
        return error(ret_stat);
    }

    if ((ret_stat = apr_socket_create(&so_listen, sa->family, SOCK_STREAM, APR_PROTO_TCP,
                                      mem_pool))
        != APR_SUCCESS)
    {
        apr_pool_destroy(mem_pool);
        return error(ret_stat);
    }

    apr_socket_opt_set(so_listen, APR_SO_NONBLOCK, 0);
    apr_socket_timeout_set(so_listen, -1);
    apr_socket_opt_set(so_listen, APR_SO_REUSEADDR, 1);

    if ((ret_stat = apr_socket_bind(so_listen, sa)) != APR_SUCCESS)
    {
        apr_pool_destroy(mem_pool);
        return error(ret_stat);
    }

    if ((ret_stat = apr_socket_listen(so_listen, DEFAULT_SO_BACKLOG)) != APR_SUCCESS)
    {
        apr_pool_destroy(mem_pool);
        return error(ret_stat);
    }

    while (run_status == 1)
    {
        apr_socket_t *ns;

        if ((ret_stat = apr_socket_accept(&ns, so_listen, mem_pool)) != APR_SUCCESS)
        {
            error(ret_stat);
            continue;
        }
        apr_socket_opt_set(ns, APR_SO_NONBLOCK, 0);
        apr_socket_timeout_set(ns, -1);
        apr_thread_t *thd_obj;

        if ((ret_stat = apr_thread_create(&thd_obj, NULL, process_client, ns, mem_pool)) != APR_SUCCESS)
        {
            error(ret_stat);
        }
    }

    apr_pool_destroy(mem_pool);
    return 0;
}

int error(apr_status_t stat)
{
    char errbuf[256];
    apr_strerror(stat, errbuf, sizeof(errbuf));
    printf("ERROR: %d, %s\n", stat, errbuf);
    return -1;
}

static void* APR_THREAD_FUNC process_client(apr_thread_t *thd, void* data)
{
    apr_socket_t * sock = (apr_socket_t*) data;
    char buf[BUFFSIZE];

    while (run_status == 1)
    {
        apr_size_t len = handle_client(buf, sizeof(buf));
        apr_socket_send(sock, buf, &len);
        if (len == 0)
        {
            break;
        }
        sleep(SYNC_TIME);
    }
    apr_socket_shutdown(sock, APR_SHUTDOWN_READWRITE);
    apr_socket_close(sock);

    return (void*) apr_thread_exit(thd, 0);
}
