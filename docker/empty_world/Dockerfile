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

FROM nvidia/opengl:1.0-glvnd-devel-ubuntu20.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get -qq update && apt-get -q -y install tzdata \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Install gpg
RUN apt-get update
RUN apt-get install -y gnupg2 lsb-release

# setup sources.list and keys
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros1-latest.list \
  && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install colcon for building packages
RUN apt-get -qq update && apt-get -q -y install \
  build-essential \
  wget \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# install ignition dome
RUN echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
  && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get -qq update && apt-get -q -y install \
  ignition-dome protobuf-compiler\
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

RUN mkdir -p ~/catkin_ws/src
COPY . ~/catkin_ws/src

# build the workspace
WORKDIR ~/catkin_ws
RUN [ "/bin/bash" , "-c" , \
  "colcon build \
  && apt-get -qq clean" ]

# start ignition
ENTRYPOINT [ "/bin/bash" , "-c" , \
  "source install/setup.bash \
  && ign launch lrauv_world.ign" ]
