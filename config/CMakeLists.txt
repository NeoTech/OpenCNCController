cmake_minimum_required(VERSION 3.20)

project(cnc_config
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header-only library
add_library(cnc_config INTERFACE)

target_include_directories(cnc_config INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Example/test executable
add_executable(config_test
    src/config_test.cpp
)

target_link_libraries(config_test PRIVATE cnc_config)

# Installation
include(GNUInstallDirs)

install(TARGETS cnc_config
    EXPORT cnc_config-targets
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES machine.toml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/opencnc
)
