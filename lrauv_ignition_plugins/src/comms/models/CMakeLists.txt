#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

add_library(simple_acoustic_model SHARED SimpleAcousticModel.cc)

target_link_libraries(simple_acoustic_model PUBLIC
  gz-plugin${IGN_PLUGIN_VER}::gz-plugin${IGN_PLUGIN_VER}
  acoustic_comms_support
)

install(
  TARGETS simple_acoustic_model
  DESTINATION lib
)
