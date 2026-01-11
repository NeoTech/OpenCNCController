# OpenCNC - Real-Time CNC Control System
# Top-level CMake configuration
cmake_minimum_required(VERSION 3.20)
project(OpenCNC VERSION 0.1.0 LANGUAGES C CXX)

# C++17 standard required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to Ninja generator
if(NOT CMAKE_GENERATOR MATCHES "Ninja")
    message(STATUS "Recommended: Use -G Ninja for faster builds")
endif()

# Compiler-specific flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3 -march=native)
    else()
        add_compile_options(-g -O0)
    endif()
endif()

# Options
option(OPENCNC_BUILD_TESTS "Build unit tests" ON)
option(OPENCNC_BUILD_EXAMPLES "Build example applications" ON)
option(OPENCNC_BUILD_DOCS "Build documentation" OFF)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add subdirectories
add_subdirectory(config)
add_subdirectory(gcode_parser)
add_subdirectory(trajectory_planner)
add_subdirectory(comm)
add_subdirectory(hmi)

if(OPENCNC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(OPENCNC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
include(GNUInstallDirs)
install(EXPORT OpenCNCTargets
    FILE OpenCNCTargets.cmake
    NAMESPACE OpenCNC::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenCNC
)

# Package configuration
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/OpenCNCConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OpenCNCConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenCNC
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/OpenCNCConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Print configuration summary
message(STATUS "")
message(STATUS "OpenCNC Configuration Summary:")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler:       ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Build tests:    ${OPENCNC_BUILD_TESTS}")
message(STATUS "  Build examples: ${OPENCNC_BUILD_EXAMPLES}")
message(STATUS "")
