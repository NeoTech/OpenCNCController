# NGC G-Code Parser Library
cmake_minimum_required(VERSION 3.20)
project(ngc_parser VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library sources
set(NGC_PARSER_SOURCES
    src/ngc_parser.cpp
)

set(NGC_PARSER_HEADERS
    include/ngc_parser.h
    include/ngc_canon.h
)

# Create library
add_library(ngc_parser ${NGC_PARSER_SOURCES})
add_library(OpenCNC::ngc_parser ALIAS ngc_parser)

target_include_directories(ngc_parser
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(ngc_parser PUBLIC cxx_std_17)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ngc_parser PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS ngc_parser
    EXPORT OpenCNCTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${NGC_PARSER_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ngc_parser
)
