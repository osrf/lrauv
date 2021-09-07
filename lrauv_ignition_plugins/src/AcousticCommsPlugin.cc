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

#include "AcousticCommsPlugin.hh"

#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Model.hh>

#include <unordered_map>

#include "lrauv_acoustic_message.pb.h"
#include "lrauv_internal_comms.pb.h"

namespace tethys
{
//////////////////////////////////////////////////
class CommsPacket
{
  /// \brief The position from which the transmission is made
  public: ignition::math::Vector3d Position() const;

  /// \brief Time of transmission
  public: std::chrono::steady_clock::time_point TimeOfTransmission() const;

  /// \brief The target vehicle
  public: uint32_t To() const;

  /// \brief The sending vehicle
  public: uint32_t From() const;

  /// \brief The data in the message
  public: std::string Data() const;

  /// \brief External comms data which you interface with
  public: lrauv_ignition_plugins::msgs::LRAUVAcousticMessage ToExternalMsg() const;
  
  /// \brief Internal comms data which you don't interface with
  public: lrauv_ignition_plugins::msgs::LRAUVInternalComms ToInternalMsg() const;

  class CommsPacketPrivateData
  {
  public:
    ignition::math::Vector3d position;
    uint32_t to;
    uint32_t from;
    uint32_t type;
    std::chrono::steady_clock::time_point timeOfTx;
    std::string data;
  };

  private: std::unique_ptr<CommsPacketPrivateData> dataPtr;

  /// \brief Factory method
  public: static CommsPacket make(
    const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage& datapayload,
    const ignition::math::Vector3d position,
    const std::chrono::steady_clock::time_point timeOfTx
    );

  /// \brief Factory method
  public: static CommsPacket make(
    const lrauv_ignition_plugins::msgs::LRAUVInternalComms& datapayload);

  private: CommsPacket() : dataPtr(std::make_unique<CommsPacketPrivateData>())
  {};
};

//////////////////////////////////////////////////
ignition::math::Vector3d CommsPacket::Position() const
{
  return this->dataPtr->position;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::time_point CommsPacket::TimeOfTransmission() const
{
  return this->dataPtr->timeOfTx;
}

//////////////////////////////////////////////////
uint32_t CommsPacket::To() const
{
  return this->dataPtr->to;
}

//////////////////////////////////////////////////
uint32_t CommsPacket::From() const
{
  return this->dataPtr->from;
}

//////////////////////////////////////////////////
std::string CommsPacket::Data() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
lrauv_ignition_plugins::msgs::LRAUVAcousticMessage 
  CommsPacket::ToExternalMsg() const
{
  lrauv_ignition_plugins::msgs::LRAUVAcousticMessage msg;
  msg.set_data(this->dataPtr->data);
  msg.set_from(this->dataPtr->from);
  msg.set_to(this->dataPtr->to);
  //msg.set_type();
  return msg;
}

//////////////////////////////////////////////////
lrauv_ignition_plugins::msgs::LRAUVInternalComms
  CommsPacket::ToInternalMsg() const
{
  lrauv_ignition_plugins::msgs::LRAUVInternalComms msg;
  msg.set_data(this->dataPtr->data);
  msg.set_from(this->dataPtr->from);
  msg.set_to(this->dataPtr->to);

  ignition::msgs::Vector3d vector;
  vector.set_x(this->dataPtr->position.X());
  vector.set_y(this->dataPtr->position.Y());
  vector.set_z(this->dataPtr->position.Z());
  msg.set_allocated_position(&vector);

  ignition::msgs::Time time;
  auto sec = 
    std::chrono::duration_cast<std::chrono::seconds>(
      this->dataPtr->timeOfTx.time_since_epoch()
    );
  time.set_sec(sec.count());
  time.set_nsec(
    this->dataPtr->timeOfTx.time_since_epoch().count() - (long)(sec.count() * 1e9));

  ignition::msgs::Header header;
  header.set_allocated_stamp(&time);

  msg.set_allocated_header(&header);
  //msg.set_type();
  return msg;
}

//////////////////////////////////////////////////
CommsPacket CommsPacket::make(
  const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage& datapayload,
  const ignition::math::Vector3d position,
  const std::chrono::steady_clock::time_point timeOfTx)
{
  CommsPacket packet;
  packet.dataPtr->position = position;
  packet.dataPtr->to = datapayload.to();
  packet.dataPtr->from = datapayload.from();
  packet.dataPtr->data = datapayload.data();
  packet.dataPtr->position = position;
  packet.dataPtr->timeOfTx = timeOfTx;
  return packet;
}

//////////////////////////////////////////////////
CommsPacket CommsPacket::make(
  const lrauv_ignition_plugins::msgs::LRAUVInternalComms& datapayload)
{
  CommsPacket packet;
  packet.dataPtr->to = datapayload.to();
  packet.dataPtr->from = datapayload.from();
  packet.dataPtr->data = datapayload.data();
  
  ignition::math::Vector3d position;
  position.X(datapayload.position().x());
  position.Y(datapayload.position().y());
  position.Z(datapayload.position().z());
  packet.dataPtr->position = position;

  auto timeMsg = datapayload.header().stamp();
  std::chrono::nanoseconds dur((long)(timeMsg.sec() * 1e9) + timeMsg.nsec());
  std::chrono::steady_clock::time_point timeOfTx(dur);
  packet.dataPtr->timeOfTx = timeOfTx;
  return packet;
}

//////////////////////////////////////////////////
class MessageManager
{
  ////////////////////////////////////////////////
  public: void MessageReachedDestination(const CommsPacket& packet)
  {
    this->externalPublisher.Publish(packet.ToExternalMsg());
  }

  ////////////////////////////////////////////////
  private: ignition::transport::Node::Publisher externalPublisher;

  ////////////////////////////////////////////////
  friend AcousticCommsPrivateData;
  friend AcousticCommsPlugin;
};

//////////////////////////////////////////////////
class ICommsModel
{
  public: virtual void enqueue_msg(CommsPacket packet);

  public: virtual void step(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm,
    MessageManager &_messageMgr);
};


//////////////////////////////////////////////////
class AcousticCommsPrivateData
{
  /// \brief handles when a message is to be sent to another robot
  public: void OnMessageSendReq(
    const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage &_msg
  );

  /// \brief handles when a message is to be recieved from another robot
  public: void OnRecieveMessage(
    const lrauv_ignition_plugins::msgs::LRAUVInternalComms &_msg
  );

  /// \brief Internal comms topic
  public: std::string internalCommsTopic {"comms/internal"};

  /// \brief Internal comms topic
  public: std::string externalCommsTopic {"comms/external"};

  /// \brief Address of myself
  public: uint32_t address;

  /// \brief Broadcast mode
  public: bool broadcast = true;

  /// \brief Node which handles comms
  public: ignition::transport::Node node;

  /// \brief Publishers which handle the communication
  public: std::unordered_map<uint32_t, ignition::transport::Node::Publisher>
    publishers;

  /// \brief Shared pointer to communications type
  public: std::shared_ptr<ICommsModel> commsModel;

  /// \brief Publisher for external comms
  public: MessageManager externalCommsPublisher;

  /// \brief Current position
  public: ignition::math::Vector3d currentPosition;

  /// \brief Current time
  public: std::chrono::steady_clock::time_point currentTime;

  /// \brief mutex
  public: std::mutex mtx;
};

//////////////////////////////////////////////////
void AcousticCommsPrivateData::OnMessageSendReq(
  const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage &_msg)
{
  std::lock_guard<std::mutex>(this->mtx);
  if (this->broadcast)
  {
    if(publishers.count(0) == 0)
    {
      publishers[0] =
        this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInternalComms>(
          this->internalCommsTopic);
    }
    
    auto packet = 
      CommsPacket::make(_msg, this->currentPosition, this->currentTime);
    publishers[0].Publish(packet.ToInternalMsg());
  }
  else
  {
    if(publishers.count(_msg.to()) == 0)
    {
      publishers[_msg.to()] =
        this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInternalComms>(
          this->internalCommsTopic + "/" + std::to_string(_msg.to()));
    }
  }
}

//////////////////////////////////////////////////
void AcousticCommsPrivateData::OnRecieveMessage(
  const lrauv_ignition_plugins::msgs::LRAUVInternalComms &_msg)
{

}

//////////////////////////////////////////////////
AcousticCommsPlugin::AcousticCommsPlugin()
  : dataPtr(std::make_unique<AcousticCommsPrivateData>())
{

}

//////////////////////////////////////////////////
void AcousticCommsPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  auto model = ignition::gazebo::Model(_entity);

  if(!_sdf->HasElement("address"))
  {
    ignerr << "No address was defined for the element " << 
      model.Name(_ecm) << " \n";
  }
  this->dataPtr->address = _sdf->Get<uint32_t>("address");

  if(_sdf->HasElement("internal_comms_prefix"))
  {
    this->dataPtr->internalCommsTopic =
      _sdf->Get<std::string>("internal_comms_prefix");
  }

  if(_sdf->HasElement("external_comms_prefix"))
  {
    this->dataPtr->externalCommsTopic =
      _sdf->Get<std::string>("external_comms_prefix");
  }

  if(_sdf->HasElement("broadcast"))
  {
    if(_sdf->Get<bool>("broadcast") == false)
    {
      this->dataPtr->internalCommsTopic = this->dataPtr->internalCommsTopic +
        "/" + std::to_string(this->dataPtr->address);
      this->dataPtr->broadcast == false;
    }
  }

  this->dataPtr->externalCommsTopic = this->dataPtr->externalCommsTopic +
    "/" + std::to_string(this->dataPtr->address);

  this->dataPtr->node.Subscribe(
    this->dataPtr->internalCommsTopic,
    &AcousticCommsPrivateData::OnRecieveMessage,
    this->dataPtr.get());

  this->dataPtr->node.Subscribe(
    this->dataPtr->externalCommsTopic + "/tx",
    &AcousticCommsPrivateData::OnMessageSendReq,
    this->dataPtr.get());

  this->dataPtr->externalCommsPublisher.externalPublisher =
    this->dataPtr->node.Advertise<
    lrauv_ignition_plugins::msgs::LRAUVAcousticMessage>(
      this->dataPtr->externalCommsTopic + "/rx");
}

//////////////////////////////////////////////////
void AcousticCommsPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
}

}
