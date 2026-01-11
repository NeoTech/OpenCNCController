cmake_minimum_required(VERSION 3.13)

# Include Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(opencnc_hal C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(opencnc_hal
    src/main.c
)

target_include_directories(opencnc_hal PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/../common
)

target_link_libraries(opencnc_hal
    pico_stdlib
    pico_multicore
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_pio
    tinyusb_device
    tinyusb_board
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(opencnc_hal 0)
pico_enable_stdio_uart(opencnc_hal 1)

# Create UF2 file for drag-and-drop programming
pico_add_extra_outputs(opencnc_hal)
