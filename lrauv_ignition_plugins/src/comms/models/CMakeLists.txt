add_library(simpleacousticmodel SHARED SimpleAcousticModel.cc)

target_link_libraries(simpleacousticmodel
PUBLIC
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  CommsSupport
)
target_include_directories(simpleacousticmodel 
  PUBLIC ../../../include/
  PUBLIC ${CMAKE_BINARY_DIR}/proto)

install(
  TARGETS simpleacousticmodel
  DESTINATION lib
)
