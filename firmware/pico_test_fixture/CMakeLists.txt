# OpenCNC Test Fixture - Pico Display 2.0
# Hardware-in-the-loop test device for Windows HMI development

cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK (must be before project())
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico_test_fixture C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# =============================================================================
# Pimoroni Pico Libraries (vendored or fetched)
# =============================================================================

# Option 1: Use FetchContent to get Pimoroni libraries
include(FetchContent)

FetchContent_Declare(
    pimoroni_pico
    GIT_REPOSITORY https://github.com/pimoroni/pimoroni-pico.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)

# Only fetch what we need
FetchContent_GetProperties(pimoroni_pico)
if(NOT pimoroni_pico_POPULATED)
    FetchContent_Populate(pimoroni_pico)
endif()

# Add Pimoroni library paths
set(PIMORONI_PICO_PATH ${pimoroni_pico_SOURCE_DIR})
include(${PIMORONI_PICO_PATH}/pimoroni_pico_import.cmake)

# =============================================================================
# Test Fixture Executable
# =============================================================================

add_executable(pico_test_fixture
    src/main.c
    src/usb_descriptors.c
    src/protocol_handler.c
    src/emulator.c
    src/display.c
    src/button_handler.c
)

target_include_directories(pico_test_fixture PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

# =============================================================================
# Pico SDK Libraries
# =============================================================================

target_link_libraries(pico_test_fixture
    # Pico SDK core
    pico_stdlib
    pico_multicore
    pico_unique_id
    
    # Hardware peripherals
    hardware_gpio
    hardware_spi
    hardware_pwm
    hardware_dma
    hardware_timer
    
    # USB (TinyUSB)
    tinyusb_device
    tinyusb_board
    
    # Pimoroni Display 2.0
    pico_display_2
    st7789
    pico_graphics
    rgbled
    button
)

# =============================================================================
# Build Configuration
# =============================================================================

# Disable stdio over USB (we use TinyUSB CDC directly)
pico_enable_stdio_usb(pico_test_fixture 0)

# Enable stdio over UART for debug output (GPIO 0/1)
pico_enable_stdio_uart(pico_test_fixture 1)

# Generate UF2 file for drag-and-drop programming
pico_add_extra_outputs(pico_test_fixture)

# =============================================================================
# Compile Definitions
# =============================================================================

target_compile_definitions(pico_test_fixture PRIVATE
    # Test fixture identification
    OPENCNC_TEST_FIXTURE=1
    FIXTURE_VERSION_MAJOR=1
    FIXTURE_VERSION_MINOR=0
    
    # Display configuration
    DISPLAY_WIDTH=320
    DISPLAY_HEIGHT=240
    DISPLAY_UPDATE_HZ=10
    
    # Protocol configuration
    STATUS_BROADCAST_HZ=50
    EMULATOR_TICK_HZ=1000
    
    # Motion queue size
    MOTION_QUEUE_SIZE=32
)

# =============================================================================
# Build Information
# =============================================================================

message(STATUS "")
message(STATUS "=== OpenCNC Test Fixture Build ===")
message(STATUS "Pico SDK Path: ${PICO_SDK_PATH}")
message(STATUS "Pimoroni Path: ${PIMORONI_PICO_PATH}")
message(STATUS "Output: pico_test_fixture.uf2")
message(STATUS "")
