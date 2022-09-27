add_executable(test_acoustic_comms test_acoustic_comms.cc)
target_include_directories(test_acoustic_comms
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_acoustic_comms
  PUBLIC gtest_main
  PRIVATE
    ${PROJECT_NAME}_support
    lrauv_gazebo_plugins::acoustic_comms_support
    lrauv_gazebo_plugins::lrauv_gazebo_messages)
gtest_discover_tests(test_acoustic_comms)

add_executable(test_acoustic_comms_multi_vehicle test_acoustic_comms_multi_vehicle.cc)
target_include_directories(test_acoustic_comms_multi_vehicle
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_acoustic_comms_multi_vehicle
  PUBLIC gtest_main
  PRIVATE
    ${PROJECT_NAME}_support
    lrauv_gazebo_plugins::acoustic_comms_support
    lrauv_gazebo_plugins::lrauv_gazebo_messages)
gtest_discover_tests(test_acoustic_comms_multi_vehicle)

add_executable(test_ahrs test_ahrs.cc)
target_link_libraries(test_ahrs
  PUBLIC gtest_main
  PRIVATE ${PROJECT_NAME}_support)
gtest_discover_tests(test_ahrs)

add_executable(test_dvl test_dvl.cc)
target_link_libraries(test_dvl
  PUBLIC gtest_main
  PRIVATE
    ${PROJECT_NAME}_support
    lrauv_gazebo_plugins::lrauv_gazebo_messages)
gtest_discover_tests(test_dvl)

add_executable(test_dvl_acoustic_comms test_dvl_acoustic_comms.cc)
target_link_libraries(test_dvl_acoustic_comms
  PUBLIC gtest_main
  PRIVATE
    ${PROJECT_NAME}_support
    lrauv_gazebo_plugins::lrauv::gazebo_messages)
gtest_discover_tests(test_dvl_acoustic_comms)

foreach(_test
    test_battery_full_charge
    test_battery_half_charge
    test_battery_low_charge
    test_buoyancy_action
    test_drop_weight
    test_elevator_action
    test_mass_shifter
    test_propeller_action
    test_rudder_action
    test_sensor_timeinterpolation
    test_sensor)
  add_executable(${_test} ${_test}.cc)
  target_link_libraries(${_test}
    PUBLIC gtest_main
    PRIVATE
      ${PROJECT_NAME}_support
      lrauv_gazebo_plugins::lrauv_gazebo_messages)
  gtest_discover_tests(${_test})
endforeach()

add_executable(test_range_bearing test_range_bearing.cc)
target_link_libraries(test_range_bearing
  PUBLIC gtest_main
  PRIVATE
    ${PROJECT_NAME}_support
    lrauv_gazebo_plugins::lrauv_gazebo_messages)
gtest_discover_tests(test_range_bearing)
