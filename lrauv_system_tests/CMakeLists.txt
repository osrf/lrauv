#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.5)

project(lrauv_system_tests)

find_package(gz-sim7 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(lrauv_ignition_plugins REQUIRED)

# Add option to build without LRAUV integration
option(ENABLE_MISSION_TESTS
  "Enable mission tests. (Requires integration with MBARI's controllers)"
  ON)

# Build-time constants
set(PROJECT_BINARY_PATH ${CMAKE_CURRENT_BINARY_DIR})
set(PROJECT_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
configure_file(include/TestConstants.hh.in include/TestConstants.hh @ONLY)

# Fetch and configure GTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# Build test support library
add_library(${PROJECT_NAME}_support
  src/LRAUVController.cc
  src/ModelManipulator.cc
  src/ModelObserver.cc
  src/Publisher.cc
)
target_include_directories(${PROJECT_NAME}_support PUBLIC
  include ${CMAKE_CURRENT_BINARY_DIR}/include)
target_link_libraries(${PROJECT_NAME}_support PUBLIC
  gz-sim7::gz-sim7
  gz-math7::gz-math7
  gz-transport12::gz-transport12
  lrauv_ignition_plugins::lrauv_command
  lrauv_ignition_plugins::lrauv_state
  gtest)
target_compile_features(${PROJECT_NAME}_support PUBLIC cxx_std_17)
#============================================================================
# Tests
include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
