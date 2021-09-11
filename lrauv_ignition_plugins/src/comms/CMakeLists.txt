add_subdirectory(models/)

add_library(CommsSupport SHARED CommsPacket.cc)

target_link_libraries(CommsSupport
PUBLIC
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)
target_include_directories(CommsSupport 
  PUBLIC ../../include/
  PUBLIC ${CMAKE_BINARY_DIR}/proto)

install(
  TARGETS CommsSupport
  DESTINATION lib
)
