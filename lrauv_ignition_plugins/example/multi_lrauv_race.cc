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

/*
 * In each iteration, for each vehicle, generate a random fin angle and thrust
 * within reasonable limits, and send the command to the vehicle.
 *
 * Usage:
 *   $ LRAUV_multi_lrauv_race
 */

#include <chrono>
#include <thread>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "lrauv_command.pb.h"

// Fin joint limits from tethys model.sdf
double random_angle_within_limits(double min=-0.261799, double max=0.261799)
{
  return min + static_cast<float>(rand()) /
    (static_cast<float>(RAND_MAX / (max - min)));
}

// Nominal speed is thruster 300 rpm = 31.4 radians per second
double random_thrust_within_limits(double min=-31.4, double max=31.4)
{
  return min + static_cast<float>(rand()) /
    (static_cast<float>(RAND_MAX / (max - min)));
}

int main(int argc, char** argv)
{
  // Initialize random seed
  srand(time(NULL));

  std::vector<std::string> ns;
  ns.push_back("tethys");
  ns.push_back("triton");
  ns.push_back("daphne");

  ignition::transport::Node node;

  std::vector<std::string> cmdTopics;
  cmdTopics.resize(ns.size(), "");
  std::vector<ignition::transport::Node::Publisher> cmdPubs;
  cmdPubs.resize(ns.size());

  // Set up topic names and publishers
  for (int i = 0; i < ns.size(); i++)
  {
    cmdTopics[i] = ignition::transport::TopicUtils::AsValidTopic(
      ns[i] + "/command_topic");
    cmdPubs[i] = node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(
      cmdTopics[i]);
  }

  std::vector<double> rudderCmds;
  rudderCmds.resize(ns.size(), 0.0);
  std::vector<double> propellerCmds;
  propellerCmds.resize(ns.size(), 0.0);

  float artificial_speedup = 1;

  while (true)
  {
    for (int i = 0; i < ns.size(); i++)
    {
      rudderCmds[i] = random_angle_within_limits(-0.01, 0.01);

      propellerCmds[i] = random_thrust_within_limits(0,
        31.4 * artificial_speedup);

      lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
      cmdMsg.set_rudderangleaction_(rudderCmds[i]);
      cmdMsg.set_propomegaaction_(propellerCmds[i]);
      cmdMsg.set_buoyancyaction_(0.0005);
      std::cout << "Commanding " << ns[i] << " rudder angle " << rudderCmds[i]
        << " rad, thrust " << propellerCmds[i] << " rad/s" << std::endl;
      cmdPubs[i].Publish(cmdMsg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}
