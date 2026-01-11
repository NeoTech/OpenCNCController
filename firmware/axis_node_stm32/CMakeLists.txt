cmake_minimum_required(VERSION 3.22)

# STM32CubeG4 path
set(STM32CUBEG4_PATH $ENV{STM32_CUBE_PATH}/STM32CubeG4)

# Project
project(opencnc_axis_node_stm32g4 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# MCU specific flags
set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS}")

# Definitions
add_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32G474xx
    -DHSE_VALUE=8000000
)

# Debug/Release flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os")
endif()

# Include directories
include_directories(
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    ../../common/canopen
)

# Source files
file(GLOB_RECURSE SOURCES
    "Core/Src/*.c"
    "Core/Startup/*.s"
    "Drivers/STM32G4xx_HAL_Driver/Src/*.c"
)

# Exclude template files
list(FILTER SOURCES EXCLUDE REGEX ".*_template\\.c$")

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32G474RETX_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_NAME}.map,--cref")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# Executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Generate binary and hex
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
)
