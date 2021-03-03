#!/bin/bash

CURRENT_DIR=$(pwd)
SOURCE="${BASH_SOURCE[0]}"

cd $(dirname $SOURCE)
cd ../

# TODO(anyone) Pending ign-launch#93 fix in ign-launch2 and forward-port to
# ign-launch3, this should chain the original env var in the line below, as
# opposed to overwriting the original.
# https://github.com/ignitionrobotics/ign-launch/pull/93
export IGN_LAUNCH_CONFIG_PATH=$(pwd)/launch
#export IGN_LAUNCH_CONFIG_PATH=$(pwd)/launch:$IGN_LAUNCH_CONFIG_PATH
export IGN_LAUNCH_PLUGIN_PATH=$(pwd)/lib:$IGN_LAUNCH_PLUGIN_PATH
export IGN_GAZEBO_RESOURCE_PATH=$(pwd)/worlds:$IGN_GAZEBO_RESOURCE_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$(pwd)/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH

# For protobuf generated libraries
export LD_LIBRARY_PATH=$(pwd)/lib:$LD_LIBRARY_PATH

cd $CURRENT_DIR
unset CURRENT_DIR
unset SOURCE
