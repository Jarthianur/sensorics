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

#ifndef SRC_SERVER_SERVER_H_
#define SRC_SERVER_SERVER_H_

#include <apr-1/apr_general.h>
#include <apr-1/apr_pools.h>
#include <stddef.h>
#include <stdint.h>

#define SERVER_DEFAULT_PORT (7997)

/**
 * Prototype of a client handler.
 * Such a function takes a char-ptr
 * and the respective array size. The result must
 * be stored in the char-ptr and will be sent to the client.
 * @param char* target array to store data
 * @param size_t array length
 * @return chars (bytes) written to the ptr
 */
typedef size_t (*client_handler)(char*, size_t);

/**
 * Run the TCP server.
 * Blocking until signal caught.
 * @param port listen on this port
 * @param handle client handler
 * @param parent memory pool parent
 * @return status
 */
int32_t server_run(int port, client_handler handle, apr_pool_t* parent);

void server_stop();

#endif /* SRC_SERVER_SERVER_H_ */
