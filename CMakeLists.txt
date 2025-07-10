cmake_minimum_required(VERSION 3.15)
project(ThreadX_UART5_Hello C)

set(CMAKE_C_STANDARD 11)

# Path to ThreadX
set(THREADX_ROOT ${THREADX_ROOT} CACHE PATH "Path to ThreadX")
# Path to HAL drivers
set(HAL_DRIVER_DIR ${HAL_DRIVER_DIR} CACHE PATH "Path to STM32 HAL driver")

# Include ThreadX build system
add_subdirectory(${THREADX_ROOT} threadx_build)

include_directories(
    ${THREADX_ROOT}/common/inc
    ${HAL_DRIVER_DIR}/Inc
    ${HAL_DRIVER_DIR}/Inc/Legacy
)

add_executable(hello_uart5
    src/main.c
)

target_link_libraries(hello_uart5 PRIVATE tx)

# Optional: Link HAL sources as needed
# target_sources(hello_uart5 PRIVATE
#     ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_uart.c
#     ...
# )
