#!/bin/bash
cd $(dirname $0)

export PICO_SDK_PATH=~/git/pico-sdk

mkdir -p build
cd build

cmake -DPICO_BOARD_HEADER_DIRS=/home/glados/git/picoExtRef/board/ -DPICO_BOARD=pico_ocxo ..
# PICO_BOARD=pico_ocxo

make
