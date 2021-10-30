#!/bin/bash

# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation

#
# Usage:
#
# Run this script like an executable, which spawns a subshell.
# Do not source it (especially if changing IFS).
#
# $ ./unserialize_for_plotting.sh [log_directory_name] [mission_name]
#
# log_directory_name: If not specified, defaults to `Logs/latest`.
# mission_name:
#   testPitchMass | testDepthVBS | testPitchAndDepthMassVBS | testYoYoCircle
#   If not specified, default to `tmp` output directory.
#
# Example:
# $ ./unserialize_for_plotting.sh 20210811T002224 testYoYoCircle
#

# Directory of this bash script
# readlink takes care of symlinks
SCRIPT_DIR="$( cd "$( dirname "$(readlink -f ${BASH_SOURCE[0]})" )" &> /dev/null && pwd )"

# Input path: MBARI code base root
# Assumes directory structure in DockerHub image
lrauv_app_path=`readlink -f $SCRIPT_DIR/../../../lrauv-application`

# Find desired input log directory
if [[ "$#" -ge 1 ]]; then
  latest=$lrauv_app_path/Logs/$1
else
  echo "Using latest log directory"

  # Input log timestamp directory name
  latest=`readlink -f $lrauv_app_path/Logs/latest`
fi

# Base name
latest_base=${latest##*/}


# Output path: output directory. Make sure this doesn't have files you want to
# keep, or they will get overwritten!
out_path=$SCRIPT_DIR/missions
using_tmp=false

if [[ "$#" -ge 2 ]]; then
  out_path="$out_path/$2"
else
  echo "Using tmp output directory"

  out_path="$out_path/tmp"
  using_tmp=true
fi

if [[ ! -d "$out_path" ]]; then
  mkdir -p $out_path
fi


# Manipulate string to derive first part of unserialized timestamp

# Tokenize with delimiter T
# Example:
#   $ echo ${tokens[*]}
#   20210721 075313
# NOTE: Changing IFS can mess up your shell. If use this, do NOT `source` this
# file to run it. Run it like an executable:
# $ ./unserialize_for_plotting.sh
#IFS='T' tokens=( $latest_base )
# This doesn't change IFS. If your string contains spaces, you must change IFS
# or tokenize some other way.
tokens=(${latest_base//T/ })

# Concatenate the two tokens. This gives you the prefix of the unserialized file
# Example: 20210721075313
unserialized_prefix=${tokens[0]}${tokens[1]}

# Truncate last two digits. They won't be in unserialized file name.
unserialized_prefix=${unserialized_prefix::-2}

unserialized_wildcard=*.csv

echo "Unserialized prefix (T removed): $unserialized_wildcard"
echo ""


# Unserialize to csv file
# Example: ./bin/unserialize -c Logs/$latest/slate depth VerticalControl.elevatorAngleAction platform_pitch_angle "(degree)"
cmd="$lrauv_app_path/bin/unserialize -c ${latest}/slate \
  depth \
  VerticalControl.depthCmd \
  platform_propeller_rotation_rate \
  SpeedControl.propOmegaAction \
  platform_buoyancy_position \
  VerticalControl.buoyancyAction \
  platform_mass_position \
  VerticalControl.massPositionAction \
  platform_elevator_angle (degree) \
  VerticalControl.elevatorAngleAction (degree) \
  platform_pitch_angle (degree) \
  platform_rudder_angle (degree) \
  HorizontalControl.rudderAngleAction (degree)"
echo "Executing command:"
echo $cmd
echo ""
# Execute in subshell with (), because the unserialize binary exits the shell!
(exec $cmd)
echo ""

# Apply wildcard.
# Assumes there is only one file matching the wildcard
# File name is unserialized timestamp
# Example: $latest/202107210753_202107210759.csv
if [ "$using_tmp" = false ]; then
  cmd="cp $latest/$unserialized_wildcard $out_path"
else
  cmd="cp $latest/$unserialized_wildcard $out_path/tmp.csv"
fi
echo "Executing command:"
echo $cmd
exec $cmd
