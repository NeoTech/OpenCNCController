# OpenCNC HMI Library
cmake_minimum_required(VERSION 3.20)
project(opencnc_hmi VERSION 0.1.0 LANGUAGES CXX)

# Header-only interface library
add_library(opencnc_hmi INTERFACE)
add_library(OpenCNC::hmi ALIAS opencnc_hmi)

target_include_directories(opencnc_hmi INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(opencnc_hmi INTERFACE cxx_std_17)

# Link dependencies
find_package(Threads REQUIRED)
target_link_libraries(opencnc_hmi INTERFACE Threads::Threads)

# Windows-specific
if(WIN32)
    target_link_libraries(opencnc_hmi INTERFACE setupapi)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS opencnc_hmi
    EXPORT OpenCNCTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
