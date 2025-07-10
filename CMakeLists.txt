cmake_minimum_required(VERSION 3.15)
project(ThreadX_UART5_Hello C)

set(CMAKE_C_STANDARD 11)

# Path to ThreadX
set(THREADX_ROOT ${THREADX_ROOT} CACHE PATH "Path to ThreadX")
# Path to HAL drivers
set(HAL_DRIVER_DIR ${HAL_DRIVER_DIR} CACHE PATH "Path to STM32 HAL driver")
set(CMSIS_DIR ${CMSIS_DIR} CACHE PATH "Path to CMSIS device headers")
set(CMSIS_CORE_DIR ${CMSIS_CORE_DIR} CACHE PATH "Path to CMSIS core headers")

# Include ThreadX build system
add_subdirectory(${THREADX_ROOT} threadx_build)

include_directories(
    ${THREADX_ROOT}/common/inc
    ${THREADX_ROOT}/ports/cortex_m7/gnu/inc
    ${HAL_DRIVER_DIR}/Inc
    ${HAL_DRIVER_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Include
    ${CMSIS_CORE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(hello_uart5
    src/main.c
    src/tx_initialize_low_level.c
)

target_compile_definitions(hello_uart5 PRIVATE STM32H750xx)

file(GLOB HAL_SOURCES ${HAL_DRIVER_DIR}/Src/*.c)
list(FILTER HAL_SOURCES EXCLUDE REGEX "stm32h7xx_hal_timebase_.*_template.c")
list(APPEND HAL_SOURCES ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_timebase_tim_template.c)
set(CMSIS_SYSTEM_SRC ${CMSIS_DIR}/Source/Templates/system_stm32h7xx.c)
target_sources(hello_uart5 PRIVATE ${HAL_SOURCES})
target_sources(hello_uart5 PRIVATE ${CMSIS_SYSTEM_SRC})

target_link_libraries(hello_uart5 PRIVATE threadx)

# Optional: Link HAL sources as needed
# target_sources(hello_uart5 PRIVATE
#     ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_uart.c
#     ...
# )
