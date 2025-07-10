#!/bin/bash
set -e

# Default directories
THREADX_DIR=${THREADX_DIR:-/workspace/threadx}
HAL_DIR=${HAL_DIR:-/workspace/stm32h7xx_hal_driver}
BUILD_DIR=build

cmake -B$BUILD_DIR -GNinja \
    -DCMAKE_TOOLCHAIN_FILE=${THREADX_DIR}/cmake/cortex_m7.cmake \
    -DTHREADX_ROOT=${THREADX_DIR} \
    -DHAL_DRIVER_DIR=${HAL_DIR} \
    .

cmake --build $BUILD_DIR
