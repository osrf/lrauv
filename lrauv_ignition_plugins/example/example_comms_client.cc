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
 * This is a small example of usage of CommsClient to send and receive data
 * packets using the acoustic subsystem. For more info:
 * https://github.com/osrf/lrauv/wiki/Examples-and-tutorials#using-commsclient
 *
 * Usage:
 *   $ ./LRAUV_example_comms_client
 */

#include <lrauv_ignition_plugins/comms/CommsClient.hh>

using namespace tethys;
using AcousticMsg = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;

int main(int _argc, char **_argv)
{
  // For sending data
  // Bind client to address 1
  CommsClient client(1, [](const auto msg){
    // Your callback function
    // To get who the message is from
    std::cout << "From: " << msg.from();
    // To get the data call
    std::cout << "Data: " << msg.data();
  });

  AcousticMsg msg;
  // Who to send to
  msg.set_to(2);
  // From who
  msg.set_from(1);
  // `LRAUVAcousticMessage_MessageType_Other` means its a data packet
  msg.set_type(AcousticMsg::MessageType::LRAUVAcousticMessage_MessageType_Other);
  // The data
  msg.set_data("test_message");
  client.SendPacket(msg);
}
