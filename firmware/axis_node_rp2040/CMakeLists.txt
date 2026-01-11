cmake_minimum_required(VERSION 3.13)

# Pico SDK import
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(axis_node_rp2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Include common CANopen headers
set(CANOPEN_COMMON_DIR "${CMAKE_SOURCE_DIR}/../common/canopen")

add_executable(${PROJECT_NAME}
    src/main.c
    src/mcp2518fd.c
    src/canopen_slave.c
    src/stepper_control.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CANOPEN_COMMON_DIR}
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_flash
    hardware_pio
)

# Enable USB output for debugging
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Generate UF2 for drag-and-drop flashing
pico_add_extra_outputs(${PROJECT_NAME})
