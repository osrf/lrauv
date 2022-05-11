add_executable(test_acoustic_comms test_acoustic_comms.cc)
target_include_directories(test_acoustic_comms
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_acoustic_comms
  PUBLIC gtest_main
  PRIVATE
    ${PROJECT_NAME}_support ${IGN_GAZEBO_TARGET}
    lrauv_ignition_plugins::acoustic_comms_support
    lrauv_ignition_plugins::lrauv_acoustic_message
)
gtest_discover_tests(test_acoustic_comms)

add_executable(test_ahrs test_ahrs.cc)
target_link_libraries(test_ahrs
  PUBLIC gtest_main
  PRIVATE ${PROJECT_NAME}_support ${IGN_GAZEBO_TARGET}
  )
gtest_discover_tests(test_ahrs)

foreach(_test
    test_battery
    test_buoyancy_action
    test_drop_weight
    test_elevator_action
    test_mass_shifter
    test_propeller_action
    test_rudder_action
    test_sensor)
  add_executable(${_test} ${_test}.cc)
  target_link_libraries(${_test}
    PUBLIC gtest_main
    PRIVATE
      ${PROJECT_NAME}_support ${IGN_GAZEBO_TARGET}
      lrauv_ignition_plugins::lrauv_command
      lrauv_ignition_plugins::lrauv_init)
  gtest_discover_tests(${_test})
endforeach()

add_executable(test_range_bearing test_range_bearing.cc)
target_link_libraries(test_range_bearing
  PUBLIC gtest_main
  PRIVATE
    ${PROJECT_NAME}_support ${IGN_GAZEBO_TARGET}
    lrauv_ignition_plugins::lrauv_range_bearing_request
    lrauv_ignition_plugins::lrauv_range_bearing_response)
gtest_discover_tests(test_range_bearing)
