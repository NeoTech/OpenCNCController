cmake_minimum_required(VERSION 3.20)

project(opencnc_win32_example
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCNC HMI (assumes parent project)
# For standalone build, adjust path

add_executable(opencnc_win32 WIN32
    example_win32.cpp
)

target_include_directories(opencnc_win32 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../hmi/include
)

target_link_libraries(opencnc_win32 PRIVATE
    setupapi
    comctl32
)

# Copy to output
add_custom_command(TARGET opencnc_win32 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/machine.toml
        $<TARGET_FILE_DIR:opencnc_win32>/machine.toml
)
