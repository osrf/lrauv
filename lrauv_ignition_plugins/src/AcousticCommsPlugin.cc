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

#include <unordered_map>

#include <ignition/gazebo/components.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/plugin/SpecializedPluginPtr.hh>
#include <ignition/transport/Node.hh>

#include <lrauv_ignition_plugins/comms/CommsModel.hh>
#include <lrauv_ignition_plugins/comms/CommsPacket.hh>
#include <lrauv_ignition_plugins/comms/MessageManager.hh>
#include <lrauv_ignition_plugins/comms/TopicDefinitions.hh>

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

  /// \brief handles when a message is to be received from another robot
  public: void OnReceiveMessage(
    const lrauv_ignition_plugins::msgs::LRAUVInternalComms &_msg
  );

  /// \brief Internal comms topic
  public: std::string internalCommsTopic {INTERNAL_COMMS_BUS};

  /// \brief Internal comms topic
  public: std::string externalCommsTopic {EXTERNAL_COMMS_BUS};

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
  public: ICommsModel* commsModel = nullptr;

  /// \brief Publisher for external comms
  public: MessageManager externalCommsPublisher;

  /// \brief Current position
  public: ignition::math::Vector3d currentPosition;

  /// \brief Current time
  public: std::chrono::steady_clock::time_point currentTime;

  /// \brief Entity to which the transponder is bound to.
  public: ignition::gazebo::Entity linkEntity;

  /// \brief Plugin pointer
  public: ignition::plugin::SpecializedPluginPtr<ICommsModel> ModelPluginPtr;

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
void AcousticCommsPrivateData::OnReceiveMessage(
  const lrauv_ignition_plugins::msgs::LRAUVInternalComms &_msg)
{
  auto packet = CommsPacket::make(_msg);
  std::lock_guard<std::mutex> lock(this->mtx);
  this->commsModel->EnqueueMsg(packet);
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

  if (!_sdf->HasElement("address"))
  {
    ignerr << "No address was defined for the element " << 
      model.Name(_ecm) << " \n";
  }
  this->dataPtr->address = _sdf->Get<uint32_t>("address");

  if (_sdf->HasElement("internal_comms_prefix"))
  {
    this->dataPtr->internalCommsTopic =
      _sdf->Get<std::string>("internal_comms_prefix");
  }

  if (_sdf->HasElement("external_comms_prefix"))
  {
    this->dataPtr->externalCommsTopic =
      _sdf->Get<std::string>("external_comms_prefix");
  }

  if (_sdf->HasElement("broadcast"))
  {
    if (_sdf->Get<bool>("broadcast") == false)
    {
      this->dataPtr->internalCommsTopic = this->dataPtr->internalCommsTopic +
        "/" + std::to_string(this->dataPtr->address);
      this->dataPtr->broadcast = false;
    }
  }

  auto vehicleModel = ignition::gazebo::Model(_entity);
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = vehicleModel.LinkByName(_ecm, linkName);
  if(this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
  {
    ignerr << "Link " << linkName << " was not found in "
      << vehicleModel.Name(_ecm) << std::endl;
    return;
  }
  ignition::gazebo::enableComponent<ignition::gazebo::components::WorldPose>(
    _ecm, this->dataPtr->linkEntity);
  
  // Create an object that can search the system paths for the plugin libraries.
  ignition::common::SystemPaths paths;

  // Create a plugin loader
  ignition::plugin::Loader loader;

  // Add the build directory path for the plugin libraries so the SystemPaths
  // object will know to search through it.
  paths.AddPluginPaths(std::getenv("TETHYS_COMMS_MODEL"));

  if (!_sdf->HasElement("model_plugin_file"))
  {
    ignerr << "No <model_plugin_file> found." 
      << "Please specify library to load pluginfrom." 
      << std::endl;
    return;
  }

  auto pluginFileName = _sdf->Get<std::string>("model_plugin_file");
  auto pluginPath = paths.FindSharedLibrary(pluginFileName);
  if (pluginPath.empty())
  {
    ignerr << "Unable to load model " << pluginFileName 
      << "file not found" << std::endl;
    return;
  }

  if(loader.LoadLib(pluginPath).empty())
  {
    std::cout << "Failed to load " << pluginPath 
      << "as a plugin library" << std::endl;
  }

  if (!_sdf->HasElement("model_name"))
  {
    ignerr << "No <model_name> found. Which model do you want to use?"
      << std::endl;
    return;
  }
  
  auto commsModels = loader.PluginsImplementing("tethys::ICommsModel");
  auto modelName = _sdf->Get<std::string>("model_name");

  if (commsModels.count(modelName) == 0)
  {
    ignerr << modelName 
      << " does not implement tethys::ICommsModel" << std::endl;
  }

  this->dataPtr->ModelPluginPtr = loader.Instantiate(modelName);
  this->dataPtr->commsModel =
    this->dataPtr->ModelPluginPtr->QueryInterface<ICommsModel>();

  if (this->dataPtr->commsModel == nullptr)
  {
    ignerr << "Failed to load comms model plugin " << modelName 
      <<  " from " << pluginPath;
    return;
  }
  igndbg << "Loaded comms model" << modelName << " from " 
    << pluginPath << std::endl;
  this->dataPtr->commsModel->Configure(_entity, _sdf, _ecm);

  if (!_sdf->HasElement("link_name"))
  {
    ignerr << "No <link_name> was found. Please specify a link to use as "
     << "a reciever" << std::endl;
    return;
  }

  this->dataPtr->externalCommsTopic = this->dataPtr->externalCommsTopic +
    "/" + std::to_string(this->dataPtr->address);

  this->dataPtr->node.Subscribe(
    this->dataPtr->internalCommsTopic,
    &AcousticCommsPrivateData::OnReceiveMessage,
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
  if (_info.paused)
    return;

  ignition::gazebo::Link baseLink(this->dataPtr->linkEntity);
  auto pose = baseLink.WorldPose(_ecm);
  
  if (!pose.has_value())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
  this->dataPtr->currentPosition = pose->Pos();
  this->dataPtr->currentTime = 
    std::chrono::steady_clock::time_point(_info.simTime);

  if (this->dataPtr->commsModel != nullptr)
    this->dataPtr->commsModel->Step(_info, _ecm, 
      this->dataPtr->externalCommsPublisher, pose.value());
}

}

IGNITION_ADD_PLUGIN(
  tethys::AcousticCommsPlugin,
  ignition::gazebo::System,
  tethys::AcousticCommsPlugin::ISystemConfigure,
  tethys::AcousticCommsPlugin::ISystemPreUpdate)
