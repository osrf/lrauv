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

#include "BuoyancyEnginePlugin.hh"
#include <chrono>

namespace tethys
{
class BuoyancyEnginePrivateData
{
  /// \brief Callback for incoming commands
  /// \param[in] volumeSetPoint - ignition message containing the desired volume
  /// (in cc) to fill/drain bladder to.
  public: void OnCmdBuoyancyEngine(
    const ignition::msgs::Double& volumeSetPoint);

  /// \brief current volume of bladder in cc
  public: double bladderVolume = 30;

  /// \brief Maximum inflation rate in cc*s^-1
  public: double maxInflationRate = 3;

  /// \brief Set-point for volume
  public: double volumeSetPoint = 30;

  /// \brief Minimum volume of bladder in cc
  public: double minVolume = 30;

  /// \brief Maximum volume of bladder in cc
  public: double maxVolume = 990;

  /// \brief The link which the bladder is attached to
  public: ignition::gazebo::Entity linkEntity;

  /// \brief The gravitational force kg*m*s^-2
  public: double gravity = 9.81;

  /// \brief The fluid density in kg*m^-3
  public: double fluidDensity = 1000;

  /// \brief The neutral volume in cc
  public: double neutralVolume = 300;

  /// \brief Trasport node for control
  public: ignition::transport::Node node;

  /// \brief Get bladder status
  public: ignition::transport::Node::Publisher statusPub;

  

  /// \brief mutex
  public: std::mutex mtx;
};

//////////////////////////////////////////////////
void BuoyancyEnginePrivateData::OnCmdBuoyancyEngine(
  const ignition::msgs::Double& volumeSetpoint)
{
  auto volume = std::max(this->minVolume, volumeSetpoint.data());
  volume = std::min(volume, this->maxVolume);

  std::lock_guard lock(this->mtx);
  this->volumeSetPoint = volume;
  igndbg << "Updating volume " << volume <<"\n";
}

//////////////////////////////////////////////////
BuoyancyEnginePlugin::BuoyancyEnginePlugin()
{
  this->dataPtr = std::make_unique<BuoyancyEnginePrivateData>();
}

//////////////////////////////////////////////////
BuoyancyEnginePlugin::~BuoyancyEnginePlugin()
{
}

//////////////////////////////////////////////////
void BuoyancyEnginePlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  auto model = ignition::gazebo::Model(_entity);
  if (!_sdf->HasElement("link_name"))
  {
    ignerr << "Buoyancy Engine must be attached to some link."  << std::endl;
    return;
  }
  this->dataPtr->linkEntity =
    model.LinkByName(_ecm, _sdf->Get<std::string>("link_name"));

  if (_sdf->HasElement("min_volume"))
  {
    this->dataPtr->minVolume = _sdf->Get<double>("min_volume");
  }

  if (_sdf->HasElement("max_volume"))
  {
    this->dataPtr->maxVolume = _sdf->Get<double>("max_volume");
  }

  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  this->dataPtr->bladderVolume = this->dataPtr->minVolume;
  if (_sdf->HasElement("default_volume"))
  {
    this->dataPtr->bladderVolume = _sdf->Get<double>("default_volume");
    this->dataPtr->volumeSetPoint = this->dataPtr->bladderVolume;
  }

  if (_sdf->HasElement("neutral_volume"))
  {
    this->dataPtr->neutralVolume = _sdf->Get<double>("neutral_volume");
  }

  std::string cmdTopic = "/buoyancy_engine/";
  std::string statusTopic = "/buoyancy_engine/current_volume";
  if (_sdf->HasElement("namespace"))
  {
    cmdTopic = ignition::transport::TopicUtils::AsValidTopic(
      "/model/" + _sdf->Get<std::string>("namespace") + "/buoyancy_engine/");
    statusTopic = ignition::transport::TopicUtils::AsValidTopic(
      "/model/" + _sdf->Get<std::string>("namespace")
      + "/buoyancy_engine/current_volume");
  }
  igndbg << "listening on topic: " << cmdTopic <<std::endl;

  if(_sdf->HasElement("max_inflation_rate"))
  {
    this->dataPtr->maxInflationRate = _sdf->Get<double>("max_inflation_rate");
  }

  if(!this->dataPtr->node.Subscribe(cmdTopic,
    &BuoyancyEnginePrivateData::OnCmdBuoyancyEngine, this->dataPtr.get()))
  {
    ignerr << "Failed to subscribe to [" << cmdTopic << "]" << std::endl;
  }

  this->dataPtr->statusPub =
    this->dataPtr->node.Advertise<ignition::msgs::Double>(statusTopic);
}

//////////////////////////////////////////////////
void BuoyancyEnginePlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  typedef std::chrono::duration<float, std::ratio<1L, 1L>> DurationInSecs;
  auto dt = std::chrono::duration_cast<DurationInSecs>(_info.dt).count();

  ignition::msgs::Double msg;

  double forceMag;
  {
    std::lock_guard lock(this->dataPtr->mtx);
    /// Adjust the bladder volume using the pump. Assume ability to pump at
    /// max flow rate
    if (this->dataPtr->bladderVolume < this->dataPtr->volumeSetPoint)
    {
      this->dataPtr->bladderVolume +=
        std::min(
          dt*this->dataPtr->maxInflationRate,
          this->dataPtr->volumeSetPoint - this->dataPtr->bladderVolume
        );
    }
    else if(this->dataPtr->bladderVolume > this->dataPtr->volumeSetPoint)
    {
      this->dataPtr->bladderVolume -=
        std::min(
          dt*this->dataPtr->maxInflationRate,
          this->dataPtr->bladderVolume - this->dataPtr->volumeSetPoint
        );
    }

    /// Populate status message
    msg.set_data(this->dataPtr->bladderVolume);
    this->dataPtr->statusPub.Publish(msg);

    // Simply use Archimede's principal to apply a force at the desired link
    // position. We take off the neutral buoyancy element in order to simulate
    // the mass of the oil in the bladder.
    forceMag = this->dataPtr->gravity * this->dataPtr->fluidDensity
      * (this->dataPtr->bladderVolume - this->dataPtr->neutralVolume) / 1e6;
  }
  ignition::gazebo::Link link(this->dataPtr->linkEntity);
  link.AddWorldWrench(_ecm, {0, 0, forceMag}, {0, 0, 0});
}
}

IGNITION_ADD_PLUGIN(
  tethys::BuoyancyEnginePlugin,
  ignition::gazebo::System,
  tethys::BuoyancyEnginePlugin::ISystemConfigure,
  tethys::BuoyancyEnginePlugin::ISystemPreUpdate)