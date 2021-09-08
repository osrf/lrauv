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

find_package(PCL 1.2 REQUIRED)

# Build protobuf messages
add_subdirectory(proto)

#============================================================================
# Plugins

# add_lrauv_plugin (<plugin_name>
#              [PROTO] [PCL] [PRIVATE_LINK_LIBS])
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
# [ADDITIONAL_SOURCES]: Optional. Specify a list of files to be added to this
# plugin. 
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
function(add_lrauv_plugin PLUGIN)
  set(options PCL INCLUDE_COMMS)
  set(oneValueArgs)
  set(multiValueArgs PROTO PRIVATE_LINK_LIBS ADDITIONAL_SOURCES)

  cmake_parse_arguments(add_lrauv_plugin
    "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${PLUGIN} 
    SHARED src/${PLUGIN}.cc ${add_lrauv_plugin_ADDITIONAL_SOURCES})
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
  endif()

  if (add_lrauv_plugin_INCLUDE_COMMS)
    target_include_directories(${PLUGIN} PUBLIC include/)
  endif()

  # Include PCL
  if (add_lrauv_plugin_PCL)

    target_include_directories(${PLUGIN}
      PUBLIC ${PCL_INCLUDE_DIRS})

  endif()

  if (add_lrauv_plugin_EXPORT_HEADERS)
    set_target_properties(
      ${PLUGIN} PROPERTIES PUBLIC_HEADER ${add_lrauv_plugin_EXPORT_HEADERS})
    install(
      TARGETS ${PLUGIN}
      DESTINATION lib
      PUBLIC_HEADER include)
  else ()
    install(
      TARGETS ${PLUGIN}
      DESTINATION lib)
  endif()
endfunction()

add_lrauv_plugin(AcousticCommsPlugin
  INCLUDE_COMMS
  PROTO 
    lrauv_acoustic_message
    lrauv_internal_comms
  ADDITIONAL_SOURCES
    src/comms/CommsPacket.cc)
add_lrauv_plugin(BuoyancyEnginePlugin)
add_lrauv_plugin(HydrodynamicsPlugin)
add_lrauv_plugin(JointPositionPlugin)
add_lrauv_plugin(ScienceSensorsSystem
  PCL
  PRIVATE_LINK_LIBS
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ${PCL_LIBRARIES})
add_lrauv_plugin(TethysCommPlugin
  PROTO
    lrauv_state
    lrauv_command)
add_lrauv_plugin(ThrusterPlugin)
add_lrauv_plugin(TimeAnalysisPlugin)

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

  # Tests
  foreach(TEST_TARGET
    test_controller
    test_drop_weight
    test_mass_shifter)

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
    )
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    include(GoogleTest)
    gtest_discover_tests(${TEST_TARGET})
  endforeach()

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
