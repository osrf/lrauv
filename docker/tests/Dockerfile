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

FROM mbari/lrauv-ignition-sim

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
        wget

# Add Ignition's latest packages, which may be more up-to-date than the ones from the MBARI image
RUN /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
RUN /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list' && \
    /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -'

# Install the latest Ignition binaries
RUN apt-get -qq update && apt-get -q -y install \
  ignition-garden

# Install PCL
RUN apt-get update \
 && apt-get install -y \
        libpcl-dev

# Clean up apt
RUN rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

USER $USERNAME

# We're in /home/developer/lrauv_ws/src/lrauv-application, go back to ~/lrauv_ws
WORKDIR ../..

# Clean up colcon workspace
RUN rm -rf build log install src/lrauv

# Use current version of the code
COPY . src/lrauv

# Build image
RUN [ "/bin/bash" , "-c" , "colcon build --cmake-args='-DBUILD_TESTING=true'" ]

# Run tests
RUN [ "/bin/bash" , "-c" , ". ~/lrauv_ws/install/setup.sh; colcon test --event-handlers console_direct+" ]
RUN [ "/bin/bash" , "-c" , ". ~/lrauv_ws/install/setup.sh; colcon test-result" ]
