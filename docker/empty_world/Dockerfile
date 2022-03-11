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

FROM nvidia/opengl:1.0-glvnd-devel-ubuntu20.04

# This avoids keyboard interaction when asked for geographic area
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
        cmake \
        cppcheck \
        curl \
        gdb \
        git \
        gnupg2 \
        libbluetooth-dev \
        libcwiid-dev \
        libgoogle-glog-dev \
        libpcl-dev \
        libspnav-dev \
        libusb-dev \
        lsb-release \
        mercurial \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-pip \
        python3-venv \
        software-properties-common \
        sudo \
        vim \
        python3-numpy \
 && apt-get clean

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get -qq update && apt-get -q -y install tzdata \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Install Ignition build tools
# https://colcon.readthedocs.io/en/released/user/installation.html
RUN /bin/sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get -qq update && apt-get -q -y install \
  build-essential \
  wget \
  python3-vcstool \
  python3-colcon-common-extensions \
  && apt-get dist-upgrade -y \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Check out Ignition source
ENV IGN_WS /home/ign_ws
RUN mkdir -p ${IGN_WS}/src \
 && cd ${IGN_WS}/src \
 && wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-garden.yaml \
 && vcs import < collection-garden.yaml

# Install Ignition dependencies
# This parses Ignition source tree to find package dependencies
RUN /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list' \
 && /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && apt-get update \
 && apt-get install -y \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ') \
 && apt-get dist-upgrade -y

# sdformat10 optional dependencies
#RUN apt-get install -y \
#    python3-psutil \
#    doxygen

# Build Ignition
RUN cd ${IGN_WS} \
 && colcon build --merge-install
 # If needed, disable tests to speed up build
 #&& colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF

RUN mkdir -p /home/colcon_ws/src
COPY . /home/colcon_ws/src

# build the workspace
WORKDIR /home/colcon_ws
RUN [ "/bin/bash" , "-c" , \
  "source ${IGN_WS}/install/setup.bash \
  && colcon build \
  && apt-get -qq clean" ]

SHELL ["/bin/bash", "-c"]

# start ignition
ENTRYPOINT [ "/bin/bash" , "-c" , \
  "source /home/colcon_ws/install/setup.bash \
  && ign gazebo buoyant_tethys.sdf" ]
