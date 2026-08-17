# core/CMakeLists.txt
#
# Builds the ROS-free planning core and its GoogleTest unit suite.
#
# core/Makefile is unchanged and still builds the hand-rolled test binary, the
# fuzz sweep and the benchmark with nothing but g++. This file exists because
# the unit suite now uses GoogleTest and CTest, and dragging those into the
# Makefile would have made the plain-g++ path stop working for anyone without
# gtest installed. So there are two entry points on purpose:
#
#   make -C core test     - hand-rolled suite, no dependencies
#   ctest --test-dir ...  - GoogleTest suite, one case per seed
#
# GoogleTest is fetched at configure time and pinned to a tag, so a green run
# stays reproducible rather than depending on whatever the distro packages.

cmake_minimum_required(VERSION 3.14)
project(planning_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(planning_core
  src/astar.cpp
  src/dstar_lite.cpp)
target_include_directories(planning_core PUBLIC include)
target_compile_options(planning_core PRIVATE -Wall -Wextra -Wpedantic)

option(PLANNING_BUILD_TESTS "Build the GoogleTest unit tests" ON)

if(PLANNING_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()

  add_executable(planners_gtest tests/planners_gtest.cpp)
  target_link_libraries(planners_gtest PRIVATE planning_core GTest::gtest_main)
  target_include_directories(planners_gtest PRIVATE tests)
  # -Wpedantic is left off here: it fires inside GoogleTest's own macros.
  target_compile_options(planners_gtest PRIVATE -Wall -Wextra)

  include(GoogleTest)
  # Registers one CTest entry per TEST_P instantiation, so a failing scenario
  # is named by its seed and can be rerun on its own.
  gtest_discover_tests(planners_gtest)
endif()
