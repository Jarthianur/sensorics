#ifndef SRC_SERVER_SERVER_H_
#define SRC_SERVER_SERVER_H_

#include <stddef.h>

#define SERVER_DEFAULT_PORT (7997)

typedef int (*client_handler)(char*, size_t);

int server_run(int port, client_handler handle);

#endif /* SRC_SERVER_SERVER_H_ */
