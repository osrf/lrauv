/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

/**
 * Subscribe to DVL velocity tracking estimates.
 *
 * An ``ign topic -e`` command equivalent that can subscribe
 * and print DVL velocity tracking messages to console.
 *
 * Usage:
 *   $ LRAUV_example_dvl_velocity <vehicle_name>
 */

#include <iostream>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "lrauv_ignition_plugins/dvl_velocity_tracking.pb.h"

void callback(const lrauv_ignition_plugins::msgs::DVLVelocityTracking &_msg)
{
  std::cout << "---" << std::endl << _msg.DebugString();
}

int main(int _argc, char **_argv)
{
  gz::transport::Node node;
  const std::string vehicleName = _argc > 1 ? _argv[1] : "tethys";
  node.Subscribe("/" + vehicleName + "/dvl/velocity", callback);

  gz::transport::waitForShutdown();
}
