#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

find_package(Protobuf REQUIRED)

function(add_lrauv_message MESSAGE)
  add_library(${MESSAGE} SHARED ${PROJECT_NAME}/${MESSAGE}.proto)
  target_link_libraries(${MESSAGE}
    protobuf::libprotobuf
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER})
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/proto)
  protobuf_generate(
    TARGET ${MESSAGE}
    LANGUAGE cpp
    IMPORT_DIRS
      ${gz-msgs${GZ_MSGS_VER}_INCLUDE_DIRS}
      ${CMAKE_BINARY_DIR}/proto
    PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/proto
  )
  target_include_directories(${MESSAGE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/proto>
    $<INSTALL_INTERFACE:include>
  )
  install(
    TARGETS ${MESSAGE}
    EXPORT ${PROJECT_NAME}
    DESTINATION lib)
endfunction()

add_lrauv_message(lrauv_acoustic_message)
add_lrauv_message(lrauv_command)
add_lrauv_message(lrauv_init)
add_lrauv_message(lrauv_internal_comms)
add_lrauv_message(lrauv_range_bearing_request)
add_lrauv_message(lrauv_range_bearing_response)
add_lrauv_message(lrauv_state)

add_lrauv_message(dvl_range_estimate)
add_lrauv_message(dvl_kinematic_estimate)
add_lrauv_message(dvl_beam_state)
target_link_libraries(dvl_beam_state
  dvl_kinematic_estimate
  dvl_range_estimate
)
add_lrauv_message(dvl_tracking_target)
target_link_libraries(dvl_tracking_target
  dvl_kinematic_estimate
  dvl_range_estimate
)
add_lrauv_message(dvl_velocity_tracking)
target_link_libraries(dvl_velocity_tracking
  dvl_beam_state
  dvl_tracking_target
  dvl_kinematic_estimate
  dvl_range_estimate
)

install(
  DIRECTORY ${CMAKE_BINARY_DIR}/proto/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h")
