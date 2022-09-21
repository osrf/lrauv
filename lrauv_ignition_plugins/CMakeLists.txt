#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(lrauv_ignition_plugins)

include(CTest)

# Option to enable profiler
option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DIGN_PROFILER_ENABLE=1")
else()
  add_definitions("-DIGN_PROFILER_ENABLE=0")
endif()

#============================================================================
# Find dependencies
find_package(gz-cmake3 REQUIRED)

find_package(gz-sim7 REQUIRED COMPONENTS gui)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-gui7 REQUIRED)
set(GZ_GUI_VER ${gz-gui7_VERSION_MAJOR})

find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

find_package(gz-sensors7 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})
set(GZ_SENSORS gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER})

find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package (Eigen3 3.3 REQUIRED)

find_package(PCL 1.2 REQUIRED)

# Build protobuf messages
add_subdirectory(proto)

#============================================================================
# Plugins

# add_lrauv_plugin (
#     <plugin_name>
#     [PROTO] msg1 msg2 ... [PCL] [GUI] [RENDERING]
#     [PRIVATE_LINK_LIBS] [INCLUDE_COMMS])
#
# Configure and install plugins.
#
# <plugin_name> Required. Name of the plugin library, which matches source file.
#
# [PROTO]: Optional. If included, plugin will be linked against custom protobuf
# messages that are listed.
#
# [PCL]: Optional. If included, plugin will be linked against PCL.
#
# [GUI]: Optional. Include for GUI plugins.
#
# [RENDERING]: Optional. Include to link against Ignition Rendering.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
function(add_lrauv_plugin PLUGIN)
  set(options PCL GUI RENDERING SENSORS INCLUDE_COMMS)
  set(oneValueArgs)
  set(multiValueArgs PROTO PRIVATE_LINK_LIBS)

  cmake_parse_arguments(add_lrauv_plugin
    "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(PLUGIN_SOURCES src/${PLUGIN}.cc)
  set(INSTALL_LIB lib)

  # GUI plugin
  if (add_lrauv_plugin_GUI)
    QT5_WRAP_CPP(${PLUGIN}_headers_MOC src/${PLUGIN}.hh)
    QT5_ADD_RESOURCES(${PLUGIN}_RCC src/${PLUGIN}.qrc)

    set(PLUGIN_SOURCES ${PLUGIN_SOURCES}
      ${${PLUGIN}_headers_MOC}
      ${${PLUGIN}_RCC}
    )
    set(INSTALL_LIB lib/gui)
  endif()

  # All plugins
  add_library(${PLUGIN} SHARED ${PLUGIN_SOURCES})
  set_property(TARGET ${PLUGIN} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${PLUGIN}
    PUBLIC
      gz-common${GZ_COMMON_VER}::profiler
      gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
      gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    PRIVATE
      ${add_lrauv_plugin_PRIVATE_LINK_LIBS}
  )

  # Link against proto libraries
  if (add_lrauv_plugin_PROTO)
    target_link_libraries(${PLUGIN}
      PUBLIC ${add_lrauv_plugin_PROTO})
  endif()

  # Include PCL
  if (add_lrauv_plugin_PCL)
    target_include_directories(${PLUGIN}
      PUBLIC ${PCL_INCLUDE_DIRS})
  endif()

  # GUI
  if (add_lrauv_plugin_GUI)
    target_link_libraries(${PLUGIN}
      PUBLIC
        TINYXML2::TINYXML2
      PRIVATE
        gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
    )

    target_include_directories(${PLUGIN}
      PUBLIC ${PCL_INCLUDE_DIRS})
  endif()

  # Rendering
  if (add_lrauv_plugin_RENDERING)
    target_link_libraries(${PLUGIN}
      PRIVATE
        gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    )
  endif()

  install(
    TARGETS ${PLUGIN}
    DESTINATION ${INSTALL_LIB})
endfunction()

add_subdirectory(src/comms/)

add_lrauv_plugin(ControlPanelPlugin GUI
  PROTO
    lrauv_command
    lrauv_state)
add_lrauv_plugin(DopplerVelocityLog
  RENDERING
  PROTO
    dvl_beam_state
    dvl_kinematic_estimate
    dvl_range_estimate
    dvl_tracking_target
    dvl_velocity_tracking)
target_link_libraries(DopplerVelocityLog PUBLIC ${GZ_SENSORS}-rendering)
add_lrauv_plugin(DopplerVelocityLogSystem RENDERING)
target_link_libraries(DopplerVelocityLogSystem PUBLIC
  DopplerVelocityLog ${GZ_SENSORS}-rendering)
add_lrauv_plugin(HydrodynamicsPlugin)
add_lrauv_plugin(RangeBearingPlugin
  PROTO
    lrauv_range_bearing_request
    lrauv_range_bearing_response
    lrauv_acoustic_message
  PRIVATE_LINK_LIBS
    acoustic_comms_support)
add_lrauv_plugin(ReferenceAxis GUI RENDERING)
add_lrauv_plugin(SpawnPanelPlugin GUI
  PROTO
    lrauv_init)
add_lrauv_plugin(TethysCommPlugin
  PROTO
    lrauv_command
    lrauv_state)
add_lrauv_plugin(TimeAnalysisPlugin)
add_lrauv_plugin(WorldCommPlugin
  PROTO
    lrauv_init)
add_lrauv_plugin(WorldConfigPlugin GUI)

#============================================================================
# Install public headers
install(
  DIRECTORY include/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.hh")

#============================================================================
# Examples
foreach(EXAMPLE
  example_buoyancy
  example_controller
  example_comms_client
  example_comms_echo
  example_dvl_velocity
  example_elevator
  example_mass_shifter
  example_rudder
  example_spawn
  example_thruster
  keyboard_teleop
  multi_lrauv_race)

  set(EXAMPLE_EXEC LRAUV_${EXAMPLE})

  add_executable(${EXAMPLE_EXEC} example/${EXAMPLE}.cc)
  set_property(TARGET ${EXAMPLE_EXEC} PROPERTY CXX_STANDARD 17)
  target_include_directories(${EXAMPLE_EXEC} PUBLIC include)
  target_link_libraries(${EXAMPLE_EXEC} PRIVATE
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    dvl_velocity_tracking lrauv_acoustic_message lrauv_command lrauv_init
    lrauv_internal_comms lrauv_range_bearing_request lrauv_range_bearing_response
    lrauv_state)

  install(
    TARGETS ${EXAMPLE_EXEC}
    DESTINATION bin)

endforeach()

#============================================================================
# Exports
install(
  EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "share/${PROJECT_NAME}/cmake"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)
configure_file(
  "hooks/hook.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.sh" @ONLY
)

#============================================================================
# World generation
foreach (WORLD_NAME "portuguese_ledge" "dvl_at_portuguese_ledge" "empty_environment_with_tiles")
  add_custom_command(
    OUTPUT ${WORLD_NAME}_gen_cmd
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/worlds/empy_expander.py
    ${CMAKE_CURRENT_SOURCE_DIR}/worlds/${WORLD_NAME}.sdf.em
    ${CMAKE_CURRENT_BINARY_DIR}/worlds/${WORLD_NAME}.sdf
  )

  add_custom_target(${WORLD_NAME}_gen_target ALL
    DEPENDS ${WORLD_NAME}_gen_cmd
  )

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/worlds/${WORLD_NAME}.sdf
    DESTINATION share/${PROJECT_NAME}/worlds)
endforeach()
#============================================================================
# Resources
install(
  DIRECTORY
    data
    models
    worlds
    ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})
