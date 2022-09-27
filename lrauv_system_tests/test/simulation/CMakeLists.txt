add_executable(test_spawn test_spawn.cc)
target_include_directories(test_spawn
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(test_spawn
  PUBLIC gtest_main
  PRIVATE
    lrauv_gazebo_plugins::lrauv_gazebo_messages
    ${PROJECT_NAME}_support
)
gtest_discover_tests(test_spawn)

add_executable(test_state test_state.cc)
target_include_directories(test_state
  PUBLIC ${CMAKE_BINARY_DIR}/proto)
target_link_libraries(test_state
  PUBLIC gtest_main
  PRIVATE
    lrauv_gazebo_plugins::lrauv_gazebo_messages
    ${PROJECT_NAME}_support
)
gtest_discover_tests(test_state)
