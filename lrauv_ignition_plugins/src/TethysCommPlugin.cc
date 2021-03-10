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

#include <chrono>

#include <ignition/plugin/Register.hh>

#include "lrauv_command.pb.h"
#include "lrauv_state.pb.h"

#include "TethysCommPlugin.hh"

using namespace tethys_comm_plugin;

void AddAngularVelocityComponent(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::AngularVelocity());
  }
    // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldAngularVelocity());
  }
}

void AddWorldPose (
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldPose());
  }
    // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldPose());
  }
}



TethysCommPlugin::TethysCommPlugin()
{
}

void TethysCommPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
 
  ignmsg << "TethysCommPlugin::Configure" << std::endl;

  // Parse SDF parameters
  if (_sdf->HasElement("command_topic"))
  {
    this->commandTopic = _sdf->Get<std::string>("command_topic");
  }
  if (_sdf->HasElement("state_topic"))
  {
    this->stateTopic = _sdf->Get<std::string>("state_topic");
  }

  // Initialize transport
  if (!this->node.Subscribe(this->commandTopic,
      &TethysCommPlugin::CommandCallback, this))
  {
    ignerr << "Error subscribing to topic " << "[" << commandTopic << "]. "
      << std::endl;
    return;
  }

  this->statePub =
    this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVState>(
    this->stateTopic);
  if (!this->statePub)
  {
    ignerr << "Error advertising topic [" << stateTopic << "]"
      << std::endl;
  }

  this->elapsed = std::chrono::steady_clock::now();

  SetupEntities(_entity, _sdf, _ecm, _eventMgr);
}

void TethysCommPlugin::SetupEntities( 
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
  if (_sdf->HasElement("model_link"))
  {
    this->baseLinkName = _sdf->Get<std::string>("model_link");
  }

  if (_sdf->HasElement("propeller_link"))
  {
    this->propellerLinkName = _sdf->Get<std::string>("propeller_link");
  }

  if (_sdf->HasElement("rudder_link"))
  {
    this->elevatorLinkName = _sdf->Get<std::string>("rudder_link");
  }

  if (_sdf->HasElement("elavator_link"))
  {
    this->propellerLinkName = _sdf->Get<std::string>("elavator_link");
  }

  auto model = ignition::gazebo::Model(_entity);
  
  this->modelLink = model.LinkByName(_ecm, this->baseLinkName);
  this->rudderLink = model.LinkByName(_ecm, this->rudderLinkName);
  this->elevatorLink = model.LinkByName(_ecm, this->elevatorLinkName);
  this->propellerLink = model.LinkByName(_ecm, this->propellerLinkName);

  AddAngularVelocityComponent(this->propellerLink, _ecm);
  AddWorldPose(this->modelLink, _ecm);
  AddWorldPose(this->rudderLink, _ecm);
  AddWorldPose(this->elevatorLink, _ecm);
}



void TethysCommPlugin::CommandCallback(
  const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg)
{
  ignmsg << "Received command: " << _msg.propomega_() << std::endl;
}

void TethysCommPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  // ignmsg << "TethysCommPlugin::PreUpdate" << std::endl;
}

void TethysCommPlugin::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  // ignmsg << "TethysCommPlugin::PostUpdate" << std::endl;

  if (std::chrono::steady_clock::now() - this->elapsed
      > std::chrono::milliseconds(100))
  {
     ignition::gazebo::Link baseLink(modelLink);
    auto model_pose = worldPose(modelLink, _ecm);
    // Publish state
    lrauv_ignition_plugins::msgs::LRAUVState stateMsg;
    auto rph = model_pose.Rot().Euler();
    ignition::msgs::Vector3d* rph_msg = new ignition::msgs::Vector3d(ignition::msgs::Convert(rph));
    stateMsg.set_allocated_rph_(rph_msg);
    stateMsg.set_depth_(-model_pose.Pos().Z());
    stateMsg.set_speed_(baseLink.WorldLinearVelocity(_ecm)->Length());

    auto latlon = sphericalCoords.SphericalFromLocalPosition(model_pose.Pos());
    stateMsg.set_latitudedeg_(latlon.X());
    stateMsg.set_longitudedeg_(latlon.Y());

    this->statePub.Publish(stateMsg);
    //ignmsg << "Published state: " << stateMsg.propomega_() << std::endl;

    this->elapsed = std::chrono::steady_clock::now();
  }
}

IGNITION_ADD_PLUGIN(
  tethys_comm_plugin::TethysCommPlugin,
  ignition::gazebo::System,
  tethys_comm_plugin::TethysCommPlugin::ISystemConfigure,
  tethys_comm_plugin::TethysCommPlugin::ISystemPreUpdate,
  tethys_comm_plugin::TethysCommPlugin::ISystemPostUpdate)
