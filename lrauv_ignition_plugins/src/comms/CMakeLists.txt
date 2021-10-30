#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

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
add_dependencies(CommsSupport
  lrauv_acoustic_message
  lrauv_internal_comms)

install(
  TARGETS CommsSupport
  DESTINATION lib
)
