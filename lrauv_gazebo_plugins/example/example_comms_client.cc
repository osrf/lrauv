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

#include <lrauv_gazebo_plugins/comms/CommsClient.hh>

using namespace tethys;
using AcousticMsg = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;

using namespace std::literals::chrono_literals;

int main(int _argc, char **_argv)
{
  // For sending data
  // Bind sender to address 1
  constexpr int senderAddress = 1;
  CommsClient sender(senderAddress, [](const auto){});

  bool messageReceived = false;
  std::mutex messageArrivalMutex;
  std::condition_variable messageArrival;

  // For receiving data
  // Bind receiver to address 2
  constexpr int receiverAddress = 2;
  CommsClient receiver(receiverAddress, [&](const auto msg)
  {
    // Your callback function
    // To get who the message is from
    std::cout << "From: " << msg.from() << std::endl;
    // To get the data call
    std::cout << "Data: " << msg.data() << std::endl;

    std::lock_guard<std::mutex> lock(messageArrivalMutex);
    messageReceived = true;
    messageArrival.notify_all();
  });

  AcousticMsg msg;
  // Who to send to
  msg.set_to(receiverAddress);
  // From who
  msg.set_from(senderAddress);
  // Message type
  msg.set_type(AcousticMsg::MessageType::LRAUVAcousticMessage_MessageType_Other);
  // Adding the data
  msg.set_data("test_message");
  sender.SendPacket(msg);

  using namespace std::literals::chrono_literals;
  std::unique_lock<std::mutex> lock(messageArrivalMutex);
  if (!messageArrival.wait_for(
      lock, 5s, [&] { return messageReceived; }))
  {
    std::cout << "5s timeout, message not received." << std::endl;
  }
}
