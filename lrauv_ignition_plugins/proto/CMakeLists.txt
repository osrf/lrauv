#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

find_package(Protobuf REQUIRED)



function(add_lrauv_message MESSAGE)
  add_library(${MESSAGE} SHARED ${MESSAGE}.proto)
  target_link_libraries(${MESSAGE}
    protobuf::libprotobuf
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER})
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/proto/${PROJECT_NAME})
  protobuf_generate(
    TARGET ${MESSAGE}
    LANGUAGE cpp
    IMPORT_DIRS ${ignition-msgs${IGN_MSGS_VER}_INCLUDE_DIRS}
    PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/proto/${PROJECT_NAME}
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

install(
  DIRECTORY ${CMAKE_BINARY_DIR}/proto/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h")
