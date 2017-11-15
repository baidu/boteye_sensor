#!/bin/bash

# auto_generate version.h
./mkcompile_h.sh
# export FX3 sdk path
build_path="$(pwd)"
export FX3FWROOT=${build_path%/*}"/cyfx3sdk"

# make firmware clean
make clean
# make firmware
make
# use elf2img to transform img format
$FX3FWROOT/util/elf2img/elf2img -i cyfxuvc.elf -o cyfxuvc.img
# make host bin test file
cd ./host_bin
./build_host_bin.sh
