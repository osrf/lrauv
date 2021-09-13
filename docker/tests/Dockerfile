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

FROM ubuntu:20.04

# This avoids keyboard interaction when asked for geographic area
ARG DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update \
 && apt-get install -y \
        build-essential \
        curl \
        gnupg2 \
        lsb-release \
        tzdata \
        wget

# Add ROS packages for colcon
RUN /bin/sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Add Ignition nightly packages
RUN /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list' && \
    /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -'

# Install colcon and Ignition
RUN apt-get -qq update && apt-get -q -y install \
  python3-colcon-common-extensions \
  ignition-fortress

# Install PCL
RUN apt-get update \
 && apt-get install -y \
        libpcl-dev

# Clean up apt
RUN rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Build LRAUV
RUN mkdir -p /home/colcon_ws/src
COPY . /home/colcon_ws/src

WORKDIR /home/colcon_ws
RUN [ "/bin/bash" , "-c" , "colcon build --cmake-args='-DBUILD_TESTING=true'" ]

# Run tests
ENTRYPOINT [ "/bin/bash" , "-c" , "colcon test" ]
