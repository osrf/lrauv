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

/*
 * In each iteration, for each vehicle, generate a random fin angle and thrust
 * within reasonable limits, and send the command to the vehicle.
 *
 * Usage:
 *   * Launch gazebo sim using : 
 *     $ gz sim -v 4 -r multi_lrauv_acoustic_demo.sdf
 *   * In another terminal, run the demo executable:
 *     $ LRAUV_multi_lrauv_acoustic_demo
 */

#include <chrono>
#include <thread>

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include "lrauv_ignition_plugins/lrauv_command.pb.h"
#include <lrauv_ignition_plugins/lrauv_acoustic_message.pb.h>

#include <lrauv_ignition_plugins/comms/CommsClient.hh>
#include <lrauv_ignition_plugins/comms/CommsPacket.hh>

using namespace tethys;

/* This is a simple demonstration of the acoustic comms plugin */
/* when used with LRAUV vehicles. It consists of 3 vehicles, */
/* Triton, Daphne, and Tethys floating side by side. Triton send */
/* a move command using acoustic comms, to the other 2 vehicles, */
/* which start moving on receiving the command. The speed of sound */
/* is purposely slowed down here to shw that the middle vehicle (Daphne) */
/* will receive the signal earlier than Tethys. */

/*    ┌─┐            ┌─┐           ┌─┐ */
/*    │ │            │ │           │ │ */
/*    │ │            │ │           │ │ */
/*    │ │            │ │           │ │ */
/*    └─┘            └─┘           └─┘ */
/*   Triton         Daphne        Tethys */

int main(int argc, char** argv)
{
  // Initialize random seed
  srand(time(NULL));

  std::vector<std::string> ns;
  ns.push_back("triton");
  ns.push_back("daphne");
  ns.push_back("tethys");

  gz::transport::Node node;

  std::vector<std::string> cmdTopics;
  cmdTopics.resize(ns.size(), "");
  std::vector<gz::transport::Node::Publisher> cmdPubs;
  cmdPubs.resize(ns.size());

  // Set up topic names and publishers for velocity command.
  for (int i = 0; i < ns.size(); i++)
  {
    cmdTopics[i] = gz::transport::TopicUtils::AsValidTopic(
      ns[i] + "/command_topic");
    cmdPubs[i] = node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(
      cmdTopics[i]);
  }

  std::vector<double> propellerCmds = {20, 0 ,0};

  // Set up publishers and callbacks for comms topics.
  auto senderAddressTriton = 2;
  CommsClient sender(senderAddressTriton, [](const auto){});

  auto receiverAddressTethys = 1;
  CommsClient receiverTethys(receiverAddressTethys, [&](const auto message)
  {
    std::cout << "Tethys received move command !" << std::endl;
    if (message.data() == "START")
    {
      propellerCmds[2] = 20;
    }
  });

  auto receiverAddressDaphne = 3;
  CommsClient receiverDaphne(receiverAddressDaphne, [&](const auto message)
  {
    std::cout << "Daphne received move command !" << std::endl;
    if (message.data() == "START")
    {
      propellerCmds[1] = 20;
    }
  });

  // Send move command message from triton
  LRAUVAcousticMessage message1;
  message1.set_to(receiverAddressTethys);
  message1.set_from(senderAddressTriton);
  message1.set_type(LRAUVAcousticMessageType);
  message1.set_data("START");

  LRAUVAcousticMessage message2;
  message2.set_to(receiverAddressDaphne);
  message2.set_from(senderAddressTriton);
  message2.set_type(LRAUVAcousticMessageType);
  message2.set_data("START");

  while (true)
  {
    if (propellerCmds[1] == 0 && propellerCmds[2] == 0)
    {
      sender.SendPacket(message1);
      sender.SendPacket(message2);
    }

    std::cout << "--------------------" << std::endl;
    std::cout << "Commanded speeds: " << std::endl;
    for (int i = 0; i < ns.size(); i++)
    {
      lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
      cmdMsg.set_rudderangleaction_(0);
      cmdMsg.set_propomegaaction_(propellerCmds[i]);
      std::cout << ns[i] << " : " << propellerCmds[i] << std::endl;
      cmdMsg.set_buoyancyaction_(0.0005);
      cmdPubs[i].Publish(cmdMsg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}
