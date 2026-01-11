cmake_minimum_required(VERSION 3.20)

# Examples directory - builds all example applications

# Win32 example (Windows only)
if(WIN32)
    add_subdirectory(win32)
endif()

# Qt5 example (if Qt5 found)
find_package(Qt5 QUIET COMPONENTS Core Widgets)
if(Qt5_FOUND)
    add_subdirectory(qt5)
    message(STATUS "Building Qt5 example")
else()
    message(STATUS "Qt5 not found - skipping Qt5 example")
endif()

# Console example (always build)
# add_subdirectory(console)
