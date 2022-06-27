#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

add_subdirectory(models/)

add_library(acoustic_comms_support SHARED CommsPacket.cc)

target_link_libraries(acoustic_comms_support PUBLIC
  gz-plugin${IGN_PLUGIN_VER}::gz-plugin${IGN_PLUGIN_VER}
  gz-sim${IGN_GAZEBO_VER}::gz-sim${IGN_GAZEBO_VER}
  lrauv_acoustic_message
  lrauv_internal_comms
)
target_include_directories(acoustic_comms_support PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS acoustic_comms_support
  EXPORT ${PROJECT_NAME}
  DESTINATION lib
)
