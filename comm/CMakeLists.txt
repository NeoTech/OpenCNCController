# Communication Layer Library
cmake_minimum_required(VERSION 3.20)
project(cnc_comm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library sources
set(COMM_SOURCES
    src/cnc_comm.cpp
)

set(COMM_HEADERS
    include/cnc_comm.h
    include/cnc_protocol.h
)

# Create library
add_library(cnc_comm ${COMM_SOURCES})
add_library(OpenCNC::cnc_comm ALIAS cnc_comm)

target_include_directories(cnc_comm
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(cnc_comm PUBLIC cxx_std_17)

# Threading
find_package(Threads REQUIRED)
target_link_libraries(cnc_comm PUBLIC Threads::Threads)

# Windows-specific
if(WIN32)
    target_link_libraries(cnc_comm PUBLIC setupapi)
endif()

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(cnc_comm PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS cnc_comm
    EXPORT OpenCNCTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${COMM_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cnc_comm
)
