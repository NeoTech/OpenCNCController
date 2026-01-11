cmake_minimum_required(VERSION 3.20)

project(opencnc_qt5_example
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Find Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_executable(opencnc_qt5
    example_qt5.cpp
)

target_include_directories(opencnc_qt5 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../hmi/include
)

target_link_libraries(opencnc_qt5 PRIVATE
    Qt5::Core
    Qt5::Widgets
    setupapi  # Windows only
)

# Copy config file
add_custom_command(TARGET opencnc_qt5 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/machine.toml
        $<TARGET_FILE_DIR:opencnc_qt5>/machine.toml
)
