#
# Copyright (C) 2022 Open Source Robotics Foundation
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
ARG BASE_IMAGE=garden

############################################################
###                        Base stage                    ###
############################################################
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu20.04 AS base

# This avoids keyboard interaction when asked for geographic area
ARG DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get -qq update && apt-get -q install -y \
    curl \
    git \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    tmux \
    tzdata \
    wget \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime

# Install Ignition build tools
# https://colcon.readthedocs.io/en/released/user/installation.html
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add -

RUN apt-get -qq update && apt-get -q -y install \
    build-essential \
    cmake \
    python3-vcstool \
    python3-colcon-common-extensions \
  && apt-get dist-upgrade -y \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Add a user with the same user_id as the user outside the container
ENV USERNAME developer
RUN addgroup --gid 1000 $USERNAME && \
  adduser --uid 1000 --ingroup $USERNAME \
    --home /home/$USERNAME --shell /bin/bash $USERNAME && \
  echo "$USERNAME:$USERNAME" | chpasswd && \
  adduser $USERNAME sudo && \
  echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.5.1/fixuid-0.5.1-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USERNAME\ngroup: $USERNAME\n" > /etc/fixuid/config.yml

RUN touch /setup.sh && chmod 0644 /setup.sh
COPY tools/setup/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

############################################################
###                  Gazebo Garden stage                 ###
############################################################
FROM base AS garden

ENV GZ_WS=/home/$USERNAME/gz_ws
RUN mkdir -p $GZ_WS/src
COPY tools/setup/gz-garden.yaml $GZ_WS/gz-garden.yaml
RUN cd $GZ_WS && vcs import src < gz-garden.yaml && vcs pull src

USER root

RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
  && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-nightly.list \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get -qq update && apt-get -q install -y \
    $(sort -u $(find $GZ_WS/src -iname "packages-$(lsb_release -cs).apt" -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') \
    libpcl-dev \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

USER $USERNAME
RUN cd $GZ_WS && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF

USER root
RUN echo ". $GZ_WS/install/setup.sh" >> /setup.sh && \
  echo "export PYTHONPATH=$GZ_WS/install/lib/python" >> /setup.sh
RUN chown -R root:root ${GZ_WS}

USER $USERNAME

############################################################
###             LRAUV simulation baseline stage          ###
############################################################
FROM ${BASE_IMAGE} AS lrauv-base

USER root
RUN apt-get -qq update && apt-get -q install -y \
    libeigen3-dev \
    libpcl-dev \
    python3-empy \
    python3-numpy \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

USER $USERNAME

ENV LRAUV_WS=/home/$USERNAME/lrauv_ws
RUN mkdir -p $LRAUV_WS/src
COPY --chown=$USERNAME . $LRAUV_WS/src/lrauv
WORKDIR $LRAUV_WS

# Run tests by default
COPY --chown=$USERNAME tools/setup/build-then-test.sh $LRAUV_WS/build-then-test.sh
CMD $LRAUV_WS/build-then-test.sh

############################################################
###                  LRAUV simulation stage              ###
############################################################
FROM lrauv-base AS lrauv

RUN cd $LRAUV_WS; . /setup.sh; \
  colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF

USER root
RUN echo ". $LRAUV_WS/install/setup.sh" >> /setup.sh
RUN chown -R root:root ${LRAUV_WS}

# Run simulation by default
USER $USERNAME
CMD gz sim -v4 buoyant_tethys.sdf
