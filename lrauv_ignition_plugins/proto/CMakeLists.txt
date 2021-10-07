find_package(Protobuf REQUIRED)

function(add_lrauv_message MESSAGE)
  add_library(${MESSAGE} SHARED ${MESSAGE}.proto)
  target_link_libraries(${MESSAGE}
    protobuf::libprotobuf
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER})
  protobuf_generate(
    TARGET ${MESSAGE}
    LANGUAGE cpp
    IMPORT_DIRS ${ignition-msgs${IGN_MSGS_VER}_INCLUDE_DIRS}
    PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/proto
  )
  install(
    TARGETS
    ${MESSAGE}
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
  DIRECTORY ${CMAKE_BINARY_DIR}
  DESTINATION include
  FILES_MATCHING
    PATTERN "*.h")
