#!/bin/bash
set -e

# Default directories
THREADX_DIR=${THREADX_DIR:-/workspace/threadx}
HAL_DIR=${HAL_DIR:-/workspace/stm32h7xx_hal_driver}
CMSIS_DIR=${CMSIS_DIR:-/workspace/cmsis_device_h7}
CMSIS_CORE_DIR=${CMSIS_CORE_DIR:-/workspace/CMSIS_5/CMSIS/Core/Include}
BUILD_DIR=build

cmake -B$BUILD_DIR -GNinja \
    -DCMAKE_TOOLCHAIN_FILE=${THREADX_DIR}/cmake/cortex_m7.cmake \
    -DTHREADX_ROOT=${THREADX_DIR} \
    -DHAL_DRIVER_DIR=${HAL_DIR} \
    -DCMSIS_DIR=${CMSIS_DIR} \
    -DCMSIS_CORE_DIR=${CMSIS_CORE_DIR} \
    .

cmake --build $BUILD_DIR
