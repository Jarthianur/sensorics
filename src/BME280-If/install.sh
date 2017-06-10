#!/bin/bash
APR_CFG=$(apr-1-config --cflags --cppflags --includes --link-ld)
#Where i2c-tools is installed from github e.g. /home/pi/i2c-tools/include/
INCLUDE_PATH=
gcc -std=c11 -o bme280-server -I $INCLUDE_PATH ../server/server.c ../i2c/i2cif.c bme280.c main.c $APR_CFG -li2c