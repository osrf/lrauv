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

#ifndef __LRAUV_IGNITION_PLUGINS_COMMS_COMMSCLIENT_HH__
#define __LRAUV_IGNITION_PLUGINS_COMMS_COMMSCLIENT_HH__

#include <functional>

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

/* #include "CommsPacket.hh" */

#include "lrauv_ignition_plugins/lrauv_acoustic_message.pb.h"

using LRAUVAcousticMessage =
    lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;
using MessageType = LRAUVAcousticMessage::MessageType;
static constexpr auto LRAUVAcousticMessageType =
    MessageType::LRAUVAcousticMessage_MessageType_Other;
auto RangeRequest =
    MessageType::LRAUVAcousticMessage_MessageType_RangeRequest;
auto RangeResponse =
    MessageType::LRAUVAcousticMessage_MessageType_RangeResponse;

namespace tethys
{
//////////////////////////////////////////////////
class CommsClient
{
using CommsMsg = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;
using Callback = std::function<void(const CommsMsg&)>;

//////////////////////////////////////////////////
/// \brief Constructor
/// \param[in] _address - The address.
/// \param[in] _callback - The callback to call when a message is received
/// \param[in] _commsPrefix - Optional. Use if your public interface to acoustic
/// comms is different. 
public: CommsClient(uint32_t _address,
  Callback _callback) :
  address(_address), callback(std::move(_callback))
{
  this->transmitter =
    this->node.Advertise<gz::msgs::Dataframe>("/broker/msgs");

  this->node.Subscribe(std::to_string(address) + "/rx",
    &CommsClient::ReceivedPacket,
    this
  );
}

//////////////////////////////////////////////////
/// \brief Send a message
/// \param[in] _msg - Comms message to be sent
public: void SendPacket(const CommsMsg& _msg)
{
  // Populate a Dataframe msg based on the
  // CommsMsg content.
  gz::msgs::Dataframe message;
  message.set_src_address(
      std::to_string(_msg.from()));
  message.set_dst_address(
      std::to_string(_msg.to()));
  message.set_data(_msg.data());

  // Set message type
  auto *frame = message.mutable_header()->add_data();
  frame->set_key("msg_type");
  if (_msg.type() == LRAUVAcousticMessageType)
    frame->add_value("LRAUVAcousticMessageType");
  else if (_msg.type() == RangeRequest)
    frame->add_value("RangeRequest");
  else if (_msg.type() == RangeResponse)
    frame->add_value("RangeResponse");

  // Publish the Dataframe message
  this->transmitter.Publish(message);
}

//////////////////////////////////////////////////
/// \brief Callback handler
/// \param[in] _msg - message received
private: void ReceivedPacket(const gz::msgs::Dataframe& _msg)
{
  // Construct a CommsMsg from the received Dataframe msg.
  CommsMsg message;
  message.set_from(
      std::stoi(_msg.src_address()));
  message.set_to(
      std::stoi(_msg.dst_address()));
  message.set_data(_msg.data());

  // Set message type.
  for (int i = 0; i < _msg.header().data_size(); i++)
  {
    if (_msg.header().data(i).key() == "msg_type")
    {
      auto type = _msg.header().data(i).value().Get(0);
      if (type == "LRAUVAcousticMessageType")
        message.set_type(LRAUVAcousticMessageType);
      else if (type == "RangeResponse")
        message.set_type(RangeResponse);
      else if (type == "RangeRequest")
        message.set_type(RangeRequest);
    }
  }

  this->callback(message);
}

/// \brief Address which the client binds to
private: uint32_t address;

/// \brief Ignition Node
private: gz::transport::Node node;

/// \brief Publisher for interfacing with AcousticCommsPlugin
private: gz::transport::Node::Publisher transmitter;

/// \brief Callback
private: Callback callback;
};
}

#endif
