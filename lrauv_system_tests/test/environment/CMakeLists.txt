add_executable(test_currents test_currents.cc)
target_include_directories(test_currents
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_currents
  PUBLIC gtest_main PRIVATE ${PROJECT_NAME}_support
)
gtest_discover_tests(test_currents)