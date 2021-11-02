#!/bin/bash

CURRENT_DIR=$(pwd)
SOURCE="${BASH_SOURCE[0]}"

cd $(dirname $SOURCE)
cd ../

export IGN_GAZEBO_RESOURCE_PATH=$(pwd)/models:$IGN_GAZEBO_RESOURCE_PATH

cd $CURRENT_DIR
unset CURRENT_DIR
unset SOURCE
