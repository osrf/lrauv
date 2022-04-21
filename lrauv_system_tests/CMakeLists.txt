#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.5)

project(lrauv_system_tests)

find_package(ignition-gazebo7 REQUIRED)
find_package(ignition-math7 REQUIRED)
find_package(ignition-transport12 REQUIRED)
find_package(lrauv_ignition_plugins REQUIRED)

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
  ignition-gazebo7::ignition-gazebo7
  ignition-math7::ignition-math7
  ignition-transport12::ignition-transport12
  lrauv_ignition_plugins::lrauv_command
  lrauv_ignition_plugins::lrauv_state
  gtest)
target_compile_features(${PROJECT_NAME}_support PUBLIC cxx_std_17)
#============================================================================
# Tests
if(BUILD_TESTING)
  enable_testing()
  include(Dart)

  add_subdirectory(test)
endif()

# #============================================================================
# # Tests
# if(BUILD_TESTING)

#   # Fetch and configure GTest
#   include(FetchContent)
#   FetchContent_Declare(
#     googletest
#     URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
#   )
#   set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#   FetchContent_MakeAvailable(googletest)

#   enable_testing()
#   include(Dart)

#   # Build-time constants
#   set("PROJECT_BINARY_PATH" ${CMAKE_CURRENT_BINARY_DIR})
#   set("PROJECT_SOURCE_PATH" ${CMAKE_CURRENT_SOURCE_DIR})
#   configure_file(test/helper/TestConstants.hh.in TestConstants.hh @ONLY)

#   include(GoogleTest)

#   # Tests
#   foreach(TEST_TARGET
#     test_acoustic_comms_orientation
#     test_ahrs
#     test_buoyancy_action
#     test_controller
#     test_drop_weight
#     test_elevator
#     test_hydrodynamics_equilibrium_velocity
#     test_mass_shifter
#     test_mission_depth_vbs
#     test_mission_pitch_and_depth_mass_vbs
#     test_mission_pitch_mass
#     test_mission_yoyo_circle
#     test_rudder
#     test_spawn
#     test_state_msg)

#     add_executable(
#       ${TEST_TARGET}
#       test/${TEST_TARGET}.cc
#     )
#     target_include_directories(${TEST_TARGET}
#       PUBLIC ${CMAKE_BINARY_DIR}/proto)
#     target_link_libraries(${TEST_TARGET}
#       PUBLIC gtest_main
#       PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
#       lrauv_command
#       lrauv_init
#       lrauv_state
#       lrauv_range_bearing_request
#       lrauv_range_bearing_response
#     )
#     include_directories(${CMAKE_CURRENT_BINARY_DIR})
#     gtest_discover_tests(${TEST_TARGET})
#   endforeach()

#   add_executable(test_comms test/test_comms.cc)
#   target_include_directories(test_comms
#     PUBLIC
#     ${CMAKE_BINARY_DIR}/proto
#     include/)
#   target_link_libraries(test_comms
#     PUBLIC gtest_main
#     PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
#     lrauv_acoustic_message
#     lrauv_internal_comms
#     CommsSupport)
#   gtest_discover_tests(test_comms)

#   add_executable(test_range_bearing test/test_range_bearing.cc)
#   target_include_directories(test_range_bearing
#     PUBLIC
#     ${CMAKE_BINARY_DIR}/proto
#     include/)
#   target_link_libraries(test_range_bearing
#     PUBLIC gtest_main
#     PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
#     lrauv_range_bearing_request
#     lrauv_range_bearing_response)
#   gtest_discover_tests(test_range_bearing)
# endif()
