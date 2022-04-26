add_executable(test_hydrostatics test_hydrostatics.cc)
target_include_directories(test_hydrostatics
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_hydrostatics
  PUBLIC gtest_main PRIVATE ${PROJECT_NAME}_support
)
gtest_discover_tests(test_hydrostatics)

add_executable(test_hydrodynamics test_hydrodynamics.cc)
target_include_directories(test_hydrodynamics
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_hydrodynamics
  PUBLIC gtest_main PRIVATE ${PROJECT_NAME}_support
)
gtest_discover_tests(test_hydrodynamics)
