/*
 * main.c
 *
 *  Created on: 12.04.2017
 *      Author: julian
 */

#include <stddef.h>
#include <stdio.h>
#include "server.h"

int handle(char* buf, size_t len)
{
    printf("Did it !\n");
    return snprintf(buf, len, "Hallo\r\n");
}

int main(int argc, char** argv)
{
    server_run(SERVER_DEFAULT_PORT, handle);
    return 0;
}

