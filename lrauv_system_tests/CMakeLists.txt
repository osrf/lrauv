#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.5)

project(lrauv_system_tests)

find_package(gz-sim7 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(lrauv_gazebo_plugins REQUIRED)

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
  src/ModelManipulator.cc
  src/ModelObserver.cc
  src/Publisher.cc
)
target_include_directories(${PROJECT_NAME}_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_support PUBLIC
  gz-sim7::gz-sim7
  gz-math7::gz-math7
  gz-transport12::gz-transport12
  gz-common5::gz-common5
  lrauv_gazebo_plugins::lrauv_gazebo_messages
  gtest)
target_compile_features(${PROJECT_NAME}_support PUBLIC cxx_std_17)
#============================================================================
# Install
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hh"
)

install(
  TARGETS ${PROJECT_NAME}_support
  EXPORT ${PROJECT_NAME}
  DESTINATION lib
)

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
# Tests
include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
