#!/bin/sh
gcc -c -o uart.o uart.c
gcc -c -o modem.o modem.c
g++ -o LiUpCon  LiUpCon.cpp uart.o modem.o
