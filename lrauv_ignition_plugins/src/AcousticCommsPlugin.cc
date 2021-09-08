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

#include <lrauv_ignition_plugins/comms/CommsModel.hh>
#include <lrauv_ignition_plugins/comms/CommsPacket.hh>
#include <lrauv_ignition_plugins/comms/MessageManager.hh>

#include <unordered_map>

#include "lrauv_acoustic_message.pb.h"
#include "lrauv_internal_comms.pb.h"

namespace tethys
{

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
  auto packet = 
      CommsPacket::make(_msg, this->currentPosition, this->currentTime);

  std::lock_guard<std::mutex>(this->mtx);
  if (this->broadcast)
  {
    if(publishers.count(0) == 0)
    {
      publishers[0] =
        this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInternalComms>(
          this->internalCommsTopic);
    }
    
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

    publishers[_msg.to()].Publish(packet.ToInternalMsg());
  }
}

//////////////////////////////////////////////////
void AcousticCommsPrivateData::OnRecieveMessage(
  const lrauv_ignition_plugins::msgs::LRAUVInternalComms &_msg)
{
  auto packet = CommsPacket::make(_msg);
  std::lock_guard<std::mutex> lock(this->mtx);
  this->commsModel->enqueue_msg(packet);
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
