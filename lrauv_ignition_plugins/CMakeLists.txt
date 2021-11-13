#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(lrauv_ignition_plugins)

#============================================================================
# Find dependencies
find_package(ignition-cmake2 REQUIRED)

find_package(ignition-gazebo6 REQUIRED COMPONENTS gui)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-gui6 REQUIRED)
set(IGN_GUI_VER ${ignition-gui6_VERSION_MAJOR})

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

find_package(PCL 1.2 REQUIRED)

# Build protobuf messages
add_subdirectory(proto)

#============================================================================
# Plugins

# add_lrauv_plugin (<plugin_name>
#              [PROTO] msg1 msg2 ... [PCL] [GUI] [PRIVATE_LINK_LIBS] [INCLUDE_COMMS])
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
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
# [INCLUDE_COMMS]: Optional. Include comms header files.
function(add_lrauv_plugin PLUGIN)
  set(options PCL GUI INCLUDE_COMMS)
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
  add_library(${PLUGIN}
    SHARED ${PLUGIN_SOURCES})
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
      PUBLIC ${add_lrauv_plugin_PROTO})
    add_dependencies(${PLUGIN} ${add_lrauv_plugin_PROTO})
  endif()

  # Include comms
  if (add_lrauv_plugin_INCLUDE_COMMS)
    target_include_directories(${PLUGIN}
      PUBLIC include)
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
        ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
    )

    target_include_directories(${PLUGIN}
      PUBLIC ${PCL_INCLUDE_DIRS})
  endif()

  install(
    TARGETS ${PLUGIN}
    DESTINATION ${INSTALL_LIB})
endfunction()

add_subdirectory(src/comms/)

add_lrauv_plugin(AcousticCommsPlugin
  INCLUDE_COMMS
  PROTO
    lrauv_acoustic_message
    lrauv_internal_comms
  PRIVATE_LINK_LIBS
    CommsSupport)

add_lrauv_plugin(HydrodynamicsPlugin)
add_lrauv_plugin(RangeBearingPlugin
  INCLUDE_COMMS
  PROTO
    lrauv_range_bearing_request
    lrauv_range_bearing_response
    lrauv_acoustic_message)
add_lrauv_plugin(ScienceSensorsSystem
  PCL
  PRIVATE_LINK_LIBS
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ${PCL_LIBRARIES})
add_lrauv_plugin(TethysCommPlugin
  PROTO
    lrauv_command
    lrauv_state)
add_lrauv_plugin(ThrusterPlugin)
add_lrauv_plugin(TimeAnalysisPlugin)
add_lrauv_plugin(VisualizePointCloud GUI)
add_lrauv_plugin(WorldCommPlugin
  PROTO
    lrauv_init)

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
  target_include_directories(${EXAMPLE_EXEC}
    PUBLIC ${CMAKE_BINARY_DIR}/proto)
  target_link_libraries(${EXAMPLE_EXEC}
    PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    PUBLIC lrauv_command
           lrauv_init)

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
# Tests
if(BUILD_TESTING)

  # Fetch and configure GTest
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(Dart)

  # Build-time constants
  set("PROJECT_BINARY_PATH" ${CMAKE_CURRENT_BINARY_DIR})
  set("PROJECT_SOURCE_PATH" ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(test/helper/TestConstants.hh.in TestConstants.hh @ONLY)

  include(GoogleTest)

  # Tests
  foreach(TEST_TARGET
    test_buoyancy_action
    test_controller
    test_drop_weight
    test_elevator
    test_mass_shifter
    test_mission_depth_vbs
    test_mission_pitch_mass
    test_mission_yoyo_circle
    test_rudder
    test_spawn)

    add_executable(
      ${TEST_TARGET}
      test/${TEST_TARGET}.cc
    )
    target_include_directories(${TEST_TARGET}
      PUBLIC ${CMAKE_BINARY_DIR}/proto)
    target_link_libraries(${TEST_TARGET}
      PUBLIC gtest_main
      PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
      lrauv_command
      lrauv_init
    )
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    gtest_discover_tests(${TEST_TARGET})
  endforeach()

  add_executable(test_comms test/test_comms.cc)
  target_include_directories(test_comms
    PUBLIC
    ${CMAKE_BINARY_DIR}/proto
    include/)
  target_link_libraries(test_comms
    PUBLIC gtest_main
    PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    lrauv_acoustic_message
    lrauv_internal_comms
    CommsSupport)
  gtest_discover_tests(test_comms)

  add_executable(test_range_bearing test/test_range_bearing.cc)
  target_include_directories(test_range_bearing
    PUBLIC
    ${CMAKE_BINARY_DIR}/proto
    include/)
  target_link_libraries(test_range_bearing
    PUBLIC gtest_main
    PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    lrauv_range_bearing_request
    lrauv_range_bearing_response)
  gtest_discover_tests(test_range_bearing)
endif()

#============================================================================
# Resources

install(
  DIRECTORY
    data
    launch
    models
    worlds
    ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})
