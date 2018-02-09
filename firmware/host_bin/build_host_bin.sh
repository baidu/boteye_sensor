#!/bin/sh

if [ $# -eq 0 ]; then
    gcc -o cam_reg_test cam_reg_test.c
    gcc -o spi_test spi_test.c
    gcc -o imu_test imu_test.c
    gcc -o version_test version_test.c
    gcc -o deviceID_test device_id.c
elif [ $# -eq 1 -a $1 = "clean" ]; then
    rm -rf *_test
fi
