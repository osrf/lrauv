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
 * This is a small example of usage of gazebo transport to send and receive
 * data packets using the acoustic subsystem. For more info:
 * https://github.com/osrf/lrauv/wiki/Examples-and-tutorials#direct-method
 *
 * Usage:
 *   $ ./LRAUV_example_comms_echo
 */

#include <sstream>
#include <unistd.h>
#include <gz/transport/Node.hh>
#include "lrauv_gazebo_plugins/lrauv_acoustic_message.pb.h"

int address = 1;
gz::transport::Node::Publisher transmitter;
using AcousticMsg = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;

//////////////////////////////////////////////////
/// \brief Function called each time a message is recieved by the comms subsystem.
void cb(const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage &_msg)
{
  std::cout << _msg.from() << ": " << _msg.data();

  lrauv_ignition_plugins::msgs::LRAUVAcousticMessage returnMsg;
  // Who to send to
  returnMsg.set_to(_msg.from());

  // From who
  returnMsg.set_from(address);

  // `LRAUVAcousticMessage_MessageType_Other` means its a data packet
  returnMsg.set_type(AcousticMsg::MessageType::LRAUVAcousticMessage_MessageType_Other);

  // The data
  returnMsg.set_data(_msg.data());

  // Send the data
  transmitter.Publish(returnMsg);
}

int main(int argc, char** argv)
{
  gz::transport::Node node;

  if (argc > 1)
  {
    std::istringstream ss(argv[1]);
    if (!(ss >> address))
    {
      std::cerr << "Invalid number: " << argv[1]
          << ". Using default value 1 instead." << std::endl;
    }
    else
    {
      if (!ss.eof())
      {
        std::cerr << "Trailing characters after number: " << argv[1]
            << ". Using default value 1 instead." << std::endl;
      }
    }
  }

  transmitter = node.Advertise<lrauv_ignition_plugins::msgs::LRAUVAcousticMessage>(
    "/comms/external/" + std::to_string(address) + "/tx");

  node.Subscribe(
    "/comms/external/" + std::to_string(address) + "/rx",
    cb
  );

  gz::transport::waitForShutdown();
}
