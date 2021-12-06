#!/usr/bin/env bash

#
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
#

#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR
# Build the docker image
docker build -t osrf_lrauv -f $DIR/tests/Dockerfile ..

rocker --nvidia --x11 --user --user-override-name=developer --user-preserve-home --  osrf_lrauv

