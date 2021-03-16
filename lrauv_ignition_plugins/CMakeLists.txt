cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(lrauv_ignition_plugins)

#============================================================================
# Find dependencies

find_package(ignition-cmake2 REQUIRED)

find_package(ignition-gazebo4 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo4_VERSION_MAJOR})

find_package(ignition-launch3 REQUIRED)

find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs6_VERSION_MAJOR})

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#============================================================================
# Build

# Build protobuf messages
add_subdirectory(proto)

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

#============================================================================
# Test controller
add_executable(TestController test/dumb_controller.cc)
set_property(TARGET TestController PROPERTY CXX_STANDARD 17)
target_include_directories(TestController
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(TestController 
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC lrauv_command)

#============================================================================
# Teleop
add_executable(TeleopController test/teleop.cc)
set_property(TARGET TeleopController PROPERTY CXX_STANDARD 17)
target_include_directories(TeleopController 
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(TeleopController  
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC lrauv_command)

#============================================================================
# Install

install(
  TARGETS
    TethysCommPlugin
    ThrusterPlugin
    lrauv_command
    lrauv_state
  DESTINATION share/lib)

install(
  DIRECTORY
    hooks
    launch
    worlds
  DESTINATION share)
