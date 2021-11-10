/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
 * Control the VBS (Variable Buoyancy System).
 *
 * The neutral volume keeps the vehicle neutrally buoyant, higher volumes
 * apply an upwards force, lower ones don't apply enough force to counter gravity.
 *
 * Usage:
 *   $ LRAUV_example_buoyancy <vehicle_name> <volume_cubic_meter>
 */

#include <chrono>
#include <thread>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "lrauv_command.pb.h"

int main(int _argc, char **_argv)
{
  std::string vehicleName("tethys");
  if (_argc > 1)
  {
    vehicleName = _argv[1];
  }

  double volume{0.0004};
  if (_argc > 2)
  {
    volume = atof(_argv[2]);
  }

  ignition::transport::Node node;
  auto commandTopic = "/" + vehicleName + "/command_topic";
  auto commandPub =
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

  while (!commandPub.HasConnections())
  {
    std::cout << "Command publisher waiting for connections..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_buoyancyaction_(volume);

  // Don't release drop-weight
  cmdMsg.set_dropweightstate_(1);

  commandPub.Publish(cmdMsg);

  std::cout << "Changing VBS volume to [" << volume << "] rad" << std::endl;
}
