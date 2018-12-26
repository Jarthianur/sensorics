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

#define LOG_COMPONENT "main"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <apr_errno.h>
#include <apr_signal.h>

#include "server/server.h"
#include "server/simple_send.h"
#include "util/types.h"

/**
 * Produce WIMDA sentence and store into buff.
 */
size_t handle(char* buff, size_t len);

/**
 * Exit signal handler.
 */
void handle_signal(int signo);

/**
 * Global run status.
 * 1 = run
 * 0 = stop
 */
int run_status = 1;

basic_server server = {0, FALSE, 1234, NULL, NULL, NULL};

int main(_unused_ int argc, _unused_ char** argv)
{
    // Initialize routine
    apr_initialize();

    apr_pool_t* mem_pool;

    apr_pool_create(&mem_pool, NULL);

    // register signal handlers
    apr_signal(SIGINT, handle_signal);
    apr_signal(SIGKILL, handle_signal);
    apr_signal(SIGPIPE, SIG_IGN);

    server.handle_client = handle;

    // Run server
    SRV_run(&server, simple_send, mem_pool);

    apr_pool_destroy(mem_pool);
    apr_terminate();
    return 0;
}

size_t handle(char* buf, _unused_ size_t len)
{
    sleep(1);
    size_t a = 0;
    while (a < 10)
    {
        buf[a++] = 'a';
    }
    buf[a++] = '\r';
    buf[a++] = '\n';
    return a;
}

void handle_signal(int signo)
{
    printf("caught signal: %d\n", signo);
    run_status = 0;
    SRV_stop(&server);
}
