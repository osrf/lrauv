#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.5)

project(lrauv_description)


#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)

#============================================================================
# Model Generation
add_custom_command(
  OUTPUT modelsdf
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/description_generator.py ${CMAKE_CURRENT_SOURCE_DIR}/models/tethys/model.sdf.in ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf
)

add_custom_target(generate_model ALL
  DEPENDS modelsdf
) 

install(DIRECTORY
  models
  ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf
  DESTINATION share/${PROJECT_NAME}/models/tethys
)

#============================================================================
# Tests
if(BUILD_TESTING)

  find_package(ignition-gazebo6 REQUIRED COMPONENTS gui)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

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

  add_executable(test_hydrostatic_stability test/test_hydrostatic_stability.cc)
  target_include_directories(test_hydrostatic_stability
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_hydrostatic_stability
    PUBLIC gtest_main
    PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  )

  gtest_discover_tests(test_hydrostatic_stability)
endif()