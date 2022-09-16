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

FROM mbari/lrauv-ignition-sim:unstable

USER root

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
        wget \
        python3-empy \
        python3-numpy

RUN sudo /bin/sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
 && sudo apt-get update \
 && sudo apt-get install -y python3-vcstool python3-colcon-common-extensions

USER $USERNAME

# Check out Gazebo source
ENV GZ_WS /home/$USERNAME/gz_ws
COPY docker/base/gz-garden.yaml ${GZ_WS}/gz-garden.yaml
RUN mkdir -p ${GZ_WS}/src && cd ${GZ_WS} && \
 vcs import src < gz-garden.yaml && vcs pull src

USER root

# Install Gazebo dependencies
# This parses Gazebo source tree to find package dependencies
RUN sudo /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && sudo /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list' \
 && sudo /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -' \
 && sudo apt-get update \
 && sudo apt-get install -y \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

# Install PCL
RUN apt-get update \
 && apt-get install -y \
        libpcl-dev

# Clean up apt
RUN rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

USER $USERNAME

RUN cd ${GZ_WS} \
 && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF
ENV PYTHONPATH ${GZ_WS}/install/lib/python
