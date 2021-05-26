cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(lrauv_ignition_plugins)

#============================================================================
# Find dependencies

find_package(ignition-cmake2 REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-launch5 REQUIRED)

find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package (Eigen3 3.3 REQUIRED)

#============================================================================
# Build

# Build protobuf messages
add_subdirectory(proto)

add_library(HydrodynamicsPlugin SHARED src/HydrodynamicsPlugin.cc)
set_property(TARGET HydrodynamicsPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(HydrodynamicsPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  Eigen3::Eigen)

add_library(TethysCommPlugin SHARED src/TethysCommPlugin.cc)
set_property(TARGET TethysCommPlugin PROPERTY CXX_STANDARD 17)
target_include_directories(TethysCommPlugin
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(TethysCommPlugin
  PUBLIC ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  # Protobuf generated libraries
  PUBLIC lrauv_command
  PUBLIC lrauv_state)

add_library(ThrusterPlugin SHARED src/ThrusterPlugin.cc)
set_property(TARGET ThrusterPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(ThrusterPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})

add_library(TimeAnalysisPlugin SHARED src/TimeAnalysisPlugin.cc)
set_property(TARGET TimeAnalysisPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(TimeAnalysisPlugin
  PUBLIC ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})

#============================================================================
# Test controller
add_executable(TestController test/test_controller.cc)
set_property(TARGET TestController PROPERTY CXX_STANDARD 17)
target_include_directories(TestController
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(TestController 
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC lrauv_command)

# Test mass shifter
add_executable(TestMassShifter test/test_mass_shifter.cc)
set_property(TARGET TestMassShifter PROPERTY CXX_STANDARD 17)
target_include_directories(TestMassShifter
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(TestMassShifter
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC lrauv_command)

#============================================================================
# Teleop
add_executable(KeyboardTeleop test/keyboard_teleop.cc)
set_property(TARGET KeyboardTeleop PROPERTY CXX_STANDARD 17)
target_include_directories(KeyboardTeleop
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(KeyboardTeleop
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC lrauv_command)

#============================================================================
# Multi-LRAUV race
add_executable(MultiLRAUVRace test/multi_lrauv_race.cc)
set_property(TARGET MultiLRAUVRace PROPERTY CXX_STANDARD 17)
target_include_directories(MultiLRAUVRace
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(MultiLRAUVRace
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC lrauv_command)

#============================================================================
# Install

install(
  TARGETS
    HydrodynamicsPlugin
    TethysCommPlugin
    ThrusterPlugin
    TimeAnalysisPlugin
    lrauv_command
    lrauv_state
  DESTINATION share/lib)

install(
  DIRECTORY
    hooks
    launch
    models
    worlds
  DESTINATION share)
