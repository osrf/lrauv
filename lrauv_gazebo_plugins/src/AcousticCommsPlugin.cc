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

#include "AcousticCommsPlugin.hh"

#include <unordered_map>

#include <gz/sim/components.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Loader.hh>
#include <gz/plugin/Register.hh>
#include <gz/plugin/SpecializedPluginPtr.hh>
#include <gz/transport/Node.hh>

#include <lrauv_gazebo_plugins/comms/CommsModel.hh>
#include <lrauv_gazebo_plugins/comms/CommsPacket.hh>
#include <lrauv_gazebo_plugins/comms/MessageManager.hh>
#include <lrauv_gazebo_plugins/comms/TopicDefinitions.hh>

#include "lrauv_gazebo_plugins/lrauv_acoustic_message.pb.h"
#include "lrauv_gazebo_plugins/lrauv_internal_comms.pb.h"

namespace tethys
{

//////////////////////////////////////////////////
class AcousticCommsPrivateData
{
  /// \brief handles when a message is to be sent to another robot
  public: void OnMessageSendReq(
    const lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage &_msg
  );

  /// \brief handles when a message is to be received from another robot
  public: void OnReceiveMessage(
    const lrauv_gazebo_plugins::msgs::LRAUVInternalComms &_msg
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
  public: gz::transport::Node node;

  /// \brief Publishers which handle the communication
  public: std::unordered_map<uint32_t, gz::transport::Node::Publisher>
    publishers;

  /// \brief Shared pointer to communications type
  public: ICommsModel* commsModel = nullptr;

  /// \brief Publisher for external comms
  public: MessageManager externalCommsPublisher;

  /// \brief Current position
  public: gz::math::Vector3d currentPosition;

  /// \brief Current time
  public: std::chrono::steady_clock::time_point currentTime;

  /// \brief Timestep
  public: std::optional<std::chrono::steady_clock::duration> 
    timestep = std::nullopt;

  /// \brief Entity to which the transponder is bound to.
  public: gz::sim::Entity linkEntity;

  /// \brief Plugin pointer
  public: gz::plugin::SpecializedPluginPtr<ICommsModel> ModelPluginPtr;

  /// \brief mutex
  public: std::mutex mtx;
};

//////////////////////////////////////////////////
void AcousticCommsPrivateData::OnMessageSendReq(
  const lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage &_msg)
{
  auto packet = 
      CommsPacket::make(_msg, this->currentPosition, this->currentTime);

  std::lock_guard<std::mutex>(this->mtx);
  if (this->broadcast)
  {
    if(publishers.count(0) == 0)
    {
      publishers[0] =
        this->node.Advertise<lrauv_gazebo_plugins::msgs::LRAUVInternalComms>(
          this->internalCommsTopic);
    }
    
    publishers[0].Publish(packet.ToInternalMsg());
  }
  else
  {
    if(publishers.count(_msg.to()) == 0)
    {
      publishers[_msg.to()] =
        this->node.Advertise<lrauv_gazebo_plugins::msgs::LRAUVInternalComms>(
          this->internalCommsTopic + "/" + std::to_string(_msg.to()));
    }

    publishers[_msg.to()].Publish(packet.ToInternalMsg());
  }
}

//////////////////////////////////////////////////
void AcousticCommsPrivateData::OnReceiveMessage(
  const lrauv_gazebo_plugins::msgs::LRAUVInternalComms &_msg)
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
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/)
{
  auto model = gz::sim::Model(_entity);

  if (!_sdf->HasElement("address"))
  {
    gzerr << "No address was defined for the element " << 
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

  if (_sdf->HasElement("timestep"))
  {
    auto timestep = _sdf->Get<double>("timestep");
    this->dataPtr->timestep = 
      std::chrono::duration<long, std::nano>(
        static_cast<long>(timestep * 1e9));
  }

  auto vehicleModel = gz::sim::Model(_entity);
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = vehicleModel.LinkByName(_ecm, linkName);
  if(this->dataPtr->linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "Link " << linkName << " was not found in "
      << vehicleModel.Name(_ecm) << std::endl;
    return;
  }
  gz::sim::enableComponent<gz::sim::components::WorldPose>(
    _ecm, this->dataPtr->linkEntity);
  
  // Create an object that can search the system paths for the plugin libraries.
  gz::common::SystemPaths paths;

  // Create a plugin loader
  gz::plugin::Loader loader;

  // Add the build directory path for the plugin libraries so the SystemPaths
  // object will know to search through it.
  paths.AddPluginPaths(std::getenv("TETHYS_COMMS_MODEL"));

  if (!_sdf->HasElement("model_plugin_file"))
  {
    gzerr << "No <model_plugin_file> found." 
      << "Please specify library to load pluginfrom." 
      << std::endl;
    return;
  }

  auto pluginFileName = _sdf->Get<std::string>("model_plugin_file");
  auto pluginPath = paths.FindSharedLibrary(pluginFileName);
  if (pluginPath.empty())
  {
    gzerr << "Unable to load model " << pluginFileName 
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
    gzerr << "No <model_name> found. Which model do you want to use?"
      << std::endl;
    return;
  }
  
  auto commsModels = loader.PluginsImplementing("tethys::ICommsModel");
  auto modelName = _sdf->Get<std::string>("model_name");

  if (commsModels.count(modelName) == 0)
  {
    gzerr << modelName 
      << " does not implement tethys::ICommsModel" << std::endl;
  }

  this->dataPtr->ModelPluginPtr = loader.Instantiate(modelName);
  this->dataPtr->commsModel =
    this->dataPtr->ModelPluginPtr->QueryInterface<ICommsModel>();

  if (this->dataPtr->commsModel == nullptr)
  {
    gzerr << "Failed to load comms model plugin " << modelName 
      <<  " from " << pluginPath;
    return;
  }
  gzdbg << "Loaded comms model" << modelName << " from " 
    << pluginPath << std::endl;
  this->dataPtr->commsModel->Configure(_entity, _sdf, _ecm);

  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> was found. Please specify a link to use as "
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
    lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage>(
      this->dataPtr->externalCommsTopic + "/rx");
}

//////////////////////////////////////////////////
void AcousticCommsPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto pose = baseLink.WorldPose(_ecm);
  
  if (!pose.has_value())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
  this->dataPtr->currentPosition = pose->Pos();
  this->dataPtr->currentTime = 
    std::chrono::steady_clock::time_point(_info.simTime);

  if (this->dataPtr->commsModel == nullptr)
  {
    static bool warn = false;
    if(!warn)
    {
      gzerr << "Failed to load Comms model.\n"; 
      warn = true;
    }
    return;
  }

  if (!this->dataPtr->timestep.has_value())
  {
    // If no timestep is defined simply load the comms model and execute the
    // environmental model.
    this->dataPtr->commsModel->Step(_info, _ecm, 
      this->dataPtr->externalCommsPublisher, pose.value());
  }
  else
  {
    // Otherwise step at the specified time step till we converge on the final
    // timestep. If the time step is larger than the dt then dt will be used.
    auto endTime = this->dataPtr->currentTime + _info.dt;
    
    while (this->dataPtr->currentTime < endTime)
    {
      gz::sim::UpdateInfo info(_info);
      info.dt = std::min(this->dataPtr->timestep.value(), _info.dt);
      info.simTime = this->dataPtr->currentTime.time_since_epoch();
      this->dataPtr->commsModel->Step(info, _ecm, 
        this->dataPtr->externalCommsPublisher, pose.value());
      this->dataPtr->currentTime += info.dt;
    }
  }
}

}

GZ_ADD_PLUGIN(
  tethys::AcousticCommsPlugin,
  gz::sim::System,
  tethys::AcousticCommsPlugin::ISystemConfigure,
  tethys::AcousticCommsPlugin::ISystemPreUpdate)
