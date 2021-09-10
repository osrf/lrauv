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

/* Development of this module has been funded by the Monterey Bay Aquarium
Research Institute (MBARI) and the David and Lucile Packard Foundation */

#ifndef __LRAUV_IGNITION_PLUGINS_COMMS_COMMSCLIENT_HH__
#define __LRAUV_IGNITION_PLUGINS_COMMS_COMMSCLIENT_HH__

#include <ignition/transport/Node.hh>

#include <functional>

#include "CommsPacket.hh"
#include "TopicDefinitions.hh"
#include "lrauv_acoustic_message.pb.h"

namespace tethys
{
//////////////////////////////////////////////////
class CommsClient
{
using CommsMsg = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;
using Callback = std::function<void(const CommsMsg&)>;

public: CommsClient(uint32_t _address,
  Callback _callback,
  std::string _commsPrefix=EXTERNAL_COMMS_BUS) :
  address(_address), callback(std::move(_callback))
{
  this->transmitter =
    this->node.Advertise<CommsMsg>(
      _commsPrefix + "/" + std::to_string(address) + "/tx");


  this->node.Subscribe(
    _commsPrefix + "/" + std::to_string(address) + "/rx",
    &CommsClient::RecievedPacket,
    this
  );
}

public: void SendPacket(const CommsMsg& msg)
{
  this->transmitter.Publish(msg);
}

private: void RecievedPacket(const CommsMsg& msg)
{
  this->callback(msg);
}
private: uint32_t address;

private: ignition::transport::Node node;

private: ignition::transport::Node::Publisher transmitter;

private: Callback callback;
};
}

#endif