#include "server.h"
#include <apr-1.0/apr.h>
#include <apr-1.0/apr_errno.h>
#include <apr-1.0/apr_general.h>
#include <apr-1.0/apr_network_io.h>
#include <apr-1.0/apr_pools.h>
#include <apr-1.0/apr_thread_proc.h>
#include <stdio.h>

#define DEFAULT_SO_BACKLOG  (SOMAXCONN)
#define BUFFSIZE            (8129)

static void* APR_THREAD_FUNC server_process_client(apr_thread_t *thd, void *);
int error(apr_status_t stat);

static client_handler handle_client = NULL;

int server_run(int port, client_handler handle, apr_pool_t* parent)
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
        return error(ret_stat);
    }

    if ((ret_stat = apr_socket_create(&so_listen, sa->family, SOCK_STREAM, APR_PROTO_TCP,
                                      mem_pool))
        != APR_SUCCESS)
    {
        return error(ret_stat);
    }

    apr_socket_opt_set(so_listen, APR_SO_NONBLOCK, 0);
    apr_socket_timeout_set(so_listen, -1);
    apr_socket_opt_set(so_listen, APR_SO_REUSEADDR, 1);

    if ((ret_stat = apr_socket_bind(so_listen, sa)) != APR_SUCCESS)
    {
        return error(ret_stat);
    }

    if ((ret_stat = apr_socket_listen(so_listen, DEFAULT_SO_BACKLOG)) != APR_SUCCESS)
    {
        return error(ret_stat);
    }

    while (1)
    {
        apr_socket_t *ns;

        if ((ret_stat = apr_socket_accept(&ns, so_listen, mem_pool)) != APR_SUCCESS)
        {
            return error(ret_stat);
        }
        apr_socket_opt_set(ns, APR_SO_NONBLOCK, 0);
        apr_socket_timeout_set(ns, -1);
        apr_thread_t *thd_obj;

        if ((ret_stat = apr_thread_create(&thd_obj, NULL, server_process_client, ns,
                                          mem_pool))
            != APR_SUCCESS)
        {
            printf("Error Creating new Thread\n");
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

static void* APR_THREAD_FUNC server_process_client(apr_thread_t *thd, void* data)
{
    apr_socket_t * sock = (apr_socket_t*) data;

    while (1)
    {
        char buf[BUFFSIZE];
        apr_size_t len = sizeof(buf) - 1;
        apr_status_t rv = apr_socket_recv(sock, buf, &len);

        if (rv != APR_EOF && len > 0)
        {
            buf[len] = '\0';
            printf("Read: %s", buf);
            fflush(stdout);
            len = handle_client(buf, sizeof(buf));
            apr_socket_send(sock, buf, &len);
        }
        else
        {
            printf("Socket Closed\n");
            apr_socket_close(sock);
            break;
        }
    }
    return (void*) apr_thread_exit(thd, 0);
}
