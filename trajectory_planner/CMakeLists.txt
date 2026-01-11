# Trajectory Planner Library
cmake_minimum_required(VERSION 3.20)
project(traj_planner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library sources
set(TRAJ_SOURCES
    src/traj_planner.cpp
)

set(TRAJ_HEADERS
    include/traj_planner.h
    include/traj_segment.h
    include/traj_queue.h
)

# Create library
add_library(traj_planner ${TRAJ_SOURCES})
add_library(OpenCNC::traj_planner ALIAS traj_planner)

target_include_directories(traj_planner
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(traj_planner PUBLIC cxx_std_17)

# Threading
find_package(Threads REQUIRED)
target_link_libraries(traj_planner PUBLIC Threads::Threads)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(traj_planner PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Math library on Linux
if(UNIX AND NOT APPLE)
    target_link_libraries(traj_planner PUBLIC m)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS traj_planner
    EXPORT OpenCNCTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${TRAJ_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/traj_planner
)
