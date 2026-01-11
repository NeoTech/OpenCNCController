cmake_minimum_required(VERSION 3.20)

# Unit tests for OpenCNC components

# Find or fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Parser tests
add_executable(parser_tests
    parser_tests.cpp
)
target_link_libraries(parser_tests PRIVATE
    ngc_parser
    gtest_main
)
gtest_discover_tests(parser_tests)

# Trajectory planner tests
add_executable(planner_tests
    planner_tests.cpp
)
target_link_libraries(planner_tests PRIVATE
    traj_planner
    gtest_main
)
gtest_discover_tests(planner_tests)

# Config tests
add_executable(config_tests
    config_tests.cpp
)
target_link_libraries(config_tests PRIVATE
    cnc_config
    gtest_main
)
gtest_discover_tests(config_tests)

# Protocol tests
add_executable(protocol_tests
    protocol_tests.cpp
)
target_link_libraries(protocol_tests PRIVATE
    cnc_comm
    gtest_main
)
gtest_discover_tests(protocol_tests)
