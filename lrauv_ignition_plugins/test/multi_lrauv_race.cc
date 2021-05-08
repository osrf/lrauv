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
 * In each iteration, for each vehicle, generate a random fin angle and thrust
 * within reasonable limits, and send the command to the vehicle.
 *
 * Usage:
 *   $ MultiLRAUVRace
 */

#include <chrono>
#include <thread>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "lrauv_command.pb.h"

// Fin joint limits from tethys model.sdf
double random_angle_within_limits(double min=-0.261799, double max=0.261799)
{
  return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

// Nominal speed is thruster 300 rpm = 31.4 radians
double random_thrust_within_limits(double min=-31.4, double max=31.4)
{
  return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

int main(int argc, char** argv)
{
  // Initialize random seed
  srand (time(NULL));

  std::string ns1 = "tethys";
  std::string ns2 = "triton";
  std::string ns3 = "daphne";

  ignition::transport::Node node;
  auto commandPub1 = 
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(ns1 + "/command_topic");
  auto commandPub2 = 
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(ns2 + "/command_topic");
  auto commandPub3 = 
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(ns3 + "/command_topic");

  double rudder1 = 0, rudder2 = 0, rudder3 = 0;
  double thrust1 = 0, thrust2 = 0, thrust3 = 0;

  float artificial_speedup = 1;
  
  while (true)
  {
    rudder1 = random_angle_within_limits(-0.01, 0.01);
    thrust1 = random_thrust_within_limits(0, 31.4 * artificial_speedup);
    lrauv_ignition_plugins::msgs::LRAUVCommand cmd1;
    cmd1.set_rudderangleaction_(rudder1);
    cmd1.set_propomegaaction_(thrust1);
    //cmd1.set_elevatorangleaction_(0);
    std::cout << "Commanding " << ns1 << " rudder angle " << rudder1 << ", thrust " << thrust1 << std::endl;
    commandPub1.Publish(cmd1);
 
    rudder2 = random_angle_within_limits(-0.01, 0.01);
    thrust2 = random_thrust_within_limits(0, 31.4 * artificial_speedup);
    lrauv_ignition_plugins::msgs::LRAUVCommand cmd2;
    cmd2.set_rudderangleaction_(rudder2);
    cmd2.set_propomegaaction_(thrust2);
    //cmd2.set_elevatorangleaction_(0);
    std::cout << "Commanding " << ns2 << " rudder angle " << rudder2 << ", thrust " << thrust2 << std::endl;
    commandPub2.Publish(cmd2);
 
    rudder3 = random_angle_within_limits(-0.01, 0.01);
    thrust3 = random_thrust_within_limits(0, 31.4 * artificial_speedup);
    lrauv_ignition_plugins::msgs::LRAUVCommand cmd3;
    cmd3.set_rudderangleaction_(rudder3);
    cmd3.set_propomegaaction_(thrust3);
    //cmd3.set_elevatorangleaction_(0);
    std::cout << "Commanding " << ns3 << " rudder angle " << rudder3 << ", thrust " << thrust3 << std::endl;
    commandPub3.Publish(cmd3);
 
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}
