#!/bin/sh

if [ $# -eq 0 ]; then
    gcc -o spi_test spi_test.c
    gcc -o version_test version_test.c
elif [ $# -eq 1 -a $1 = "clean" ]; then
    rm -rf *_test
fi
