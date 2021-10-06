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
 * Usage:
 *   $ LRAUV_example_buoyancy <vehicle_name>
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

  ignition::transport::Node node;
  auto commandTopic = "/" + vehicleName + "/command_topic";
  auto commandPub =
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

  lrauv_ignition_plugins::msgs::LRAUVCommand buoyancyMessage;
  std::cout << "Expect vehicle to move up \n";
  for (int i = 0 ; i < 10; i++)
  {
    buoyancyMessage.set_buoyancyaction_(0.0004);
    commandPub.Publish(buoyancyMessage);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}
