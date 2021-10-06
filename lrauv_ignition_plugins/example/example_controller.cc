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
 * This is a stupidly simple test controller
 * that wiggles the fins a bit and then commands the robot
 * to charge forward.
 *
 * Usage:
 *   $ LRAUV_example_controller <vehicle_name>
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

  double angle = 0.17;

  // Wiggle rudder
  for (int i = 0; i < 10; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    lrauv_ignition_plugins::msgs::LRAUVCommand rudderMsg;
    angle *= -1;
    rudderMsg.set_buoyancyaction_(0.0005); // Keep it stable
    rudderMsg.set_dropweightstate_(1);
    rudderMsg.set_rudderangleaction_(angle);
    commandPub.Publish(rudderMsg);
    std::cout << "moving rudder to " << angle << "\n";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand rudderStopMsg;
  rudderStopMsg.set_buoyancyaction_(0.0005);
  rudderStopMsg.set_rudderangleaction_(0);
  rudderStopMsg.set_dropweightstate_(1);
  commandPub.Publish(rudderStopMsg);
  std::cout << "moving rudder to " << angle << "\n";

  // Wiggle elevator
  for (int i = 0; i < 10; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    lrauv_ignition_plugins::msgs::LRAUVCommand elevatorMsg;
    angle *= -1;
    elevatorMsg.set_buoyancyaction_(0.0005);
    elevatorMsg.set_elevatorangleaction_(angle);
    elevatorMsg.set_dropweightstate_(1);
    commandPub.Publish(elevatorMsg);
    std::cout << "moving elevator to " << angle << "\n";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand elevatorStopMsg;
  elevatorStopMsg.set_buoyancyaction_(0.0005);
  elevatorStopMsg.set_elevatorangleaction_(0);
  elevatorStopMsg.set_dropweightstate_(1);
  commandPub.Publish(elevatorStopMsg);
  std::cout << "moving elevator to " << angle << "\n";

  // Charge forward
  lrauv_ignition_plugins::msgs::LRAUVCommand thrustMsg;
  thrustMsg.set_buoyancyaction_(0.0005);
  thrustMsg.set_propomegaaction_(30);
  thrustMsg.set_dropweightstate_(1);
  commandPub.Publish(thrustMsg);
  std::cout << "charging forward!\n";

  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  lrauv_ignition_plugins::msgs::LRAUVCommand stopMsg;
  stopMsg.set_buoyancyaction_(0.0005);
  stopMsg.set_propomegaaction_(0);
  stopMsg.set_dropweightstate_(1);
  commandPub.Publish(stopMsg);
  std::cout << "stop\n";
}
