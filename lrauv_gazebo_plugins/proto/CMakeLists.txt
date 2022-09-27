#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

find_package(Protobuf REQUIRED)

file(GLOB PROTO_SRCS ${PROJECT_NAME}/*.proto)
add_library(lrauv_gazebo_messages SHARED ${PROTO_SRCS})
target_link_libraries(lrauv_gazebo_messages
  protobuf::libprotobuf
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER})
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/proto)
protobuf_generate(
  TARGET lrauv_gazebo_messages
  LANGUAGE cpp
  IMPORT_DIRS
    ${gz-msgs${GZ_MSGS_VER}_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}/proto
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/proto
)
target_include_directories(lrauv_gazebo_messages PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/proto>
  $<INSTALL_INTERFACE:include>
)
install(
  TARGETS lrauv_gazebo_messages
  EXPORT ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY ${CMAKE_BINARY_DIR}/proto/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
)
