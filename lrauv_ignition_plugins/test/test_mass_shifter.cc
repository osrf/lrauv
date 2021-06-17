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
 * Tests the mass shifter actuator by sending commands to shift the battery
 * forward and backward.
 *
 * Usage:
 *   $ TestMassShifter
 */

#include <chrono>
#include <thread>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "lrauv_command.pb.h"

int main(int argc, char** argv)
{
  ignition::transport::Node node;
  auto commandTopic = "/tethys/command_topic";
  auto commandPub =
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

  // Negative moves toward nose of vehicle; positive moves toward tail
  double dist = -0.001;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand batteryMsg;
  batteryMsg.set_masspositionaction_(dist);
  commandPub.Publish(batteryMsg);
  std::cout << "Commanding mass shifter to " << dist << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand batteryReverseMsg;
  batteryReverseMsg.set_masspositionaction_(-dist);
  commandPub.Publish(batteryReverseMsg);
  std::cout << "Commanding mass shifter to " << -dist << std::endl;
}
