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

ARG BASE_IMAGE=mbari/lrauv-ignition-sim:unstable
FROM ${BASE_IMAGE}

ENV LRAUV_WS /home/$USERNAME/lrauv_ws
WORKDIR ${LRAUV_WS}

# Clean up colcon workspace
RUN rm -rf build log install src/lrauv

# Use current version of the code
COPY . src/lrauv

# Build image
RUN [ "/bin/bash" , "-c" , ". ~/ign_ws/install/setup.sh; colcon build --cmake-args='-DBUILD_TESTING=true'" ]

# Drop entrypoint
ENTRYPOINT []

# Run tests
CMD [ "/bin/bash" , "-c" , ". ~/lrauv_ws/install/setup.sh; colcon test --event-handlers console_direct+ && colcon test-result" ]
