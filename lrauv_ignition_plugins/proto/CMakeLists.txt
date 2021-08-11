find_package(Protobuf REQUIRED)

add_library(lrauv_command SHARED lrauv_command.proto)
target_link_libraries(lrauv_command
  protobuf::libprotobuf
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER})
protobuf_generate(
  TARGET lrauv_command
  LANGUAGE cpp
  IMPORT_DIRS ${ignition-msgs${IGN_MSGS_VER}_INCLUDE_DIRS}
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/proto
)

add_library(lrauv_state SHARED lrauv_state.proto)
target_link_libraries(lrauv_state
  protobuf::libprotobuf
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER})
protobuf_generate(
  TARGET lrauv_state
  LANGUAGE cpp
  IMPORT_DIRS ${ignition-msgs${IGN_MSGS_VER}_INCLUDE_DIRS}
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/proto
)

install(
  TARGETS
    lrauv_command
    lrauv_state
  DESTINATION lib)

install(
  DIRECTORY ${CMAKE_BINARY_DIR}
  DESTINATION include
  FILES_MATCHING
    PATTERN "*.h")
