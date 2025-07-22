#!/bin/bash
set -e

# Default directories
# THREADX_DIR=${THREADX_DIR:-/workspace/threadx}
# HAL_DIR=${HAL_DIR:-/workspace/stm32h7xx_hal_driver}
# CMSIS_DIR=${CMSIS_DIR:-/workspace/cmsis_device_h7}
# CMSIS_CORE_DIR=${CMSIS_CORE_DIR:-/workspace/CMSIS_5/CMSIS/Core/Include}

# Optional preset argument (Debug or Release). Defaults to Debug.
PRESET=${1:-Release}

# Configure and build using CMake presets. The preset determines the build
# directory under `build/<preset>`.
cmake --preset "$PRESET"

cmake --build --preset "$PRESET"
