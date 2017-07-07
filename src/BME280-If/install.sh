#!/bin/bash
APR_CFG=$(apr-1-config --cflags --cppflags --includes --link-ld)
gcc -std=c11 -o bmeserve ../server/server.c ../i2c/i2cif.c bme280.c main.c $APR_CFG
