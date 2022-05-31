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

/**
 * Control the elevator joint position.
 *
 * Positive angles rotate the horizontal fins counter-clockwise when looking
 * from starboard, which makes the vehicle go down when propelled forward.
 * Negative commands do the opposite.
 *
 * Usage:
 *   $ LRAUV_example_elevator <vehicle_name> <angle_radians>
 */
#include <chrono>
#include <thread>

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include "lrauv_ignition_plugins/lrauv_command.pb.h"

int main(int _argc, char **_argv)
{
  std::string vehicleName("tethys");
  if (_argc > 1)
  {
    vehicleName = _argv[1];
  }

  double angle{0.17};
  if (_argc > 2)
  {
    angle = atof(_argv[2]);
  }

  gz::transport::Node node;
  auto commandTopic = "/" + vehicleName + "/command_topic";
  auto commandPub =
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

  while (!commandPub.HasConnections())
  {
    std::cout << "Command publisher waiting for connections..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_elevatorangleaction_(angle);

  // Keep it stable
  cmdMsg.set_buoyancyaction_(0.0005);
  cmdMsg.set_dropweightstate_(1);

  commandPub.Publish(cmdMsg);

  std::cout << "Moving elevator to [" << angle << "] rad" << std::endl;
}
