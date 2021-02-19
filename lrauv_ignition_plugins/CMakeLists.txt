cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Find dependencies

find_package(ignition-cmake2 REQUIRED)

project(lrauv_ignition_plugins)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo4 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo4_VERSION_MAJOR})

find_package(ignition-launch3 REQUIRED)

#============================================================================
# Build

add_library(TethysCommPlugin SHARED src/TethysCommPlugin.cc)
set_property(TARGET TethysCommPlugin PROPERTY CXX_STANDARD 17)
# TODO(mabelzhang) Temporary absolute paths, hack to find location of .pb.h, .pb.cc, .pb.o files generated in MBARI code base
target_include_directories(TethysCommPlugin PUBLIC
  /home/developer/mbari/lrauv_ws/src/lrauv-application/proto)
target_link_libraries(TethysCommPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PRIVATE /home/developer/mbari/lrauv_ws/src/lrauv-application/proto/lrauv_command.pb.o
  PRIVATE /home/developer/mbari/lrauv_ws/src/lrauv-application/proto/lrauv_state.pb.o)

add_library(ThrusterPlugin SHARED src/ThrusterPlugin.cc)
set_property(TARGET ThrusterPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(ThrusterPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})

#============================================================================
# Install

install(TARGETS
  TethysCommPlugin ThrusterPlugin
  DESTINATION share/lib)

install(DIRECTORY
  hooks
  launch
  worlds
  DESTINATION share)
