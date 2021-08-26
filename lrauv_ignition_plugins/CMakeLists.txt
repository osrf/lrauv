cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(lrauv_ignition_plugins)

#============================================================================
# Find dependencies

find_package(ignition-cmake2 REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-sensors6 REQUIRED)
set(IGN_SENSORS_VER ${ignition-sensors6_VERSION_MAJOR})

find_package(ignition-launch5 REQUIRED)

find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-utils1 REQUIRED)
set(IGN_UTILS_VER ${ignition-utils1_VERSION_MAJOR})

find_package (Eigen3 3.3 REQUIRED)

# Build protobuf messages
add_subdirectory(proto)

#============================================================================
# Plugins

# add_lrauv_plugin (<plugin_name>
#              [PROTO])
#
# Configure and install plugins.
#
# <plugin_name> Required. Name of the plugin library, which matches source file.
#
# [PROTO]: Optional. If included, plugin will be linked against custom protobuf messages.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
function(add_lrauv_plugin PLUGIN)
  set(options PROTO)
  set(oneValueArgs)
  set(multiValueArgs PRIVATE_LINK_LIBS)

  cmake_parse_arguments(add_lrauv_plugin "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  set_property(TARGET ${PLUGIN} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${PLUGIN}
    PUBLIC
      ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
      ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    PRIVATE
      ${add_lrauv_plugin_PRIVATE_LINK_LIBS}
  )

  # Link against proto libraries
  if (add_lrauv_plugin_PROTO)

    target_include_directories(${PLUGIN}
      PUBLIC ${CMAKE_BINARY_DIR}/proto)
    target_link_libraries(${PLUGIN}
      PUBLIC lrauv_command
      PUBLIC lrauv_state)

  endif()

  install(
    TARGETS ${PLUGIN}
    DESTINATION lib)

endfunction()

add_lrauv_plugin(BuoyancyEnginePlugin)
add_lrauv_plugin(HydrodynamicsPlugin)
add_lrauv_plugin(JointPositionPlugin)
add_lrauv_plugin(ScienceSensorsSystem
  PRIVATE_LINK_LIBS
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER})
add_lrauv_plugin(TethysCommPlugin PROTO)
add_lrauv_plugin(ThrusterPlugin)
add_lrauv_plugin(TimeAnalysisPlugin)

#============================================================================
# Examples
foreach(EXAMPLE
  keyboard_teleop
  multi_lrauv_race
  example_controller
  example_mass_shifter
  example_buoyancy)

  set(EXAMPLE_EXEC LRAUV_${EXAMPLE})

  add_executable(${EXAMPLE_EXEC} example/${EXAMPLE}.cc)
  set_property(TARGET ${EXAMPLE_EXEC} PROPERTY CXX_STANDARD 17)
  target_include_directories(${EXAMPLE_EXEC}
    PUBLIC ${CMAKE_BINARY_DIR}/proto)
  target_link_libraries(${EXAMPLE_EXEC}
    PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    PUBLIC lrauv_command)

  install(
    TARGETS ${EXAMPLE_EXEC}
    DESTINATION bin)

endforeach()

#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)

#============================================================================
# Resources

install(
  DIRECTORY
    launch
    models
    worlds
    ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})
