foreach(_test
    test_yoyo_circle_mission
    test_pitch_mass_mission
    test_depth_vbs_mission
    test_pitch_and_depth_mass_vbs_mission)
  add_executable(${_test} ${_test}.cc)
  target_link_libraries(${_test}
    PUBLIC gtest_main
    PRIVATE ${PROJECT_NAME}_support
  )
  gtest_discover_tests(${_test})
endforeach()
