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
 * Spawns a vehicle with a given name, at the given coordinates.
 *
 * Usage:
 *   $ LRAUV_example_spawn <vehicle_name> <latitude> <longitude>
 */

#include <stdlib.h>
#include <chrono>
#include <thread>

#include <gz/transport/Node.hh>
#include "lrauv_ignition_plugins/lrauv_init.pb.h"

int main(int _argc, char **_argv)
{
  std::string vehicleName("tethys");
  if (_argc > 1)
  {
    vehicleName = _argv[1];
  }

  double latitude{0.0};
  if (_argc > 2)
  {
    latitude = atof(_argv[2]);
  }

  double longitude{0.0};
  if (_argc > 3)
  {
    longitude = atof(_argv[3]);
  }

  gz::transport::Node node;
  auto spawnTopic = "/lrauv/init";
  auto spawnPub =
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInit>(spawnTopic);

  while (!spawnPub.HasConnections())
  {
    std::cout << "Init publisher waiting for connections..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  lrauv_ignition_plugins::msgs::LRAUVInit spawnMsg;
  spawnMsg.mutable_id_()->set_data(vehicleName);
  spawnMsg.set_initlat_(latitude);
  spawnMsg.set_initlon_(longitude);
  spawnPub.Publish(spawnMsg);
  std::cout << "Spawning [" << vehicleName << "] at latitude [" << latitude
            << "] / longitude [" << longitude << "]" << std::endl;
}
