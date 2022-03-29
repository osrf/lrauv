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

#include <chrono>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/spherical_coordinates.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/TopicUtils.hh>

#include "lrauv_init.pb.h"

#include "WorldCommPlugin.hh"

using namespace tethys;

/////////////////////////////////////////////////
void WorldCommPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
  // Parse SDF parameters
  if (_sdf->HasElement("spawn_topic"))
  {
    this->spawnTopic = _sdf->Get<std::string>("spawn_topic");
  }

  // Initialize transport
  if (!this->node.Subscribe(this->spawnTopic,
      &WorldCommPlugin::SpawnCallback, this))
  {
    ignerr << "Error subscribing to topic " << "[" << this->spawnTopic << "]. "
      << std::endl;
    return;
  }
  ignmsg << "Listening to spawn messages on [" << this->spawnTopic << "]"
         << std::endl;

  std::string worldName;
  auto worldEntity = ignition::gazebo::worldEntity(_entity, _ecm);
  if (ignition::gazebo::kNullEntity != worldEntity)
  {
    ignition::gazebo::World world(worldEntity);
    auto worldNameOpt = world.Name(_ecm);
    if (worldNameOpt)
    {
      worldName = worldNameOpt.value();
    }
    else
    {
      ignerr << "Failed to get name for world entity [" << worldEntity
             << "]" << std::endl;
    }
  }
  else
  {
    ignerr << "Failed to get world entity" << std::endl;
  }

  if (worldName.empty())
  {
    ignerr << "Failed to initialize plugin." << std::endl;
    return;
  }

  // Valid world name for services
  auto topicWorldName =
      ignition::transport::TopicUtils::AsValidTopic(worldName);
  if (topicWorldName.empty())
  {
    ignerr << "Invalid world name ["
           << worldName << "]" << std::endl;
    return;
  }

  // Services
  this->createService = "/world/" + topicWorldName + "/create";
  this->setSphericalCoordsService = "/world/" + topicWorldName
    + "/set_spherical_coordinates";
}

/////////////////////////////////////////////////
void WorldCommPlugin::ServiceResponse(const ignition::msgs::Boolean &_rep,
  const bool _result)
{
  if (!_result || !_rep.data())
    ignerr << "Error requesting some service." << std::endl;
}

/////////////////////////////////////////////////
void WorldCommPlugin::SpawnCallback(
  const lrauv_ignition_plugins::msgs::LRAUVInit &_msg)
{
  igndbg << "Received spawn message: " << std::endl
    << _msg.DebugString() << std::endl;

  if (!_msg.has_id_())
  {
    ignerr << "Received empty ID, can't initialize vehicle." << std::endl;
    return;
  }

  auto lat = _msg.initlat_();
  auto lon = _msg.initlon_();
  auto ele = -_msg.initz_();

  // Center the world around the first vehicle spawned
  if (this->spawnCount == 0)
  {
    igndbg << "Setting world origin coordinates to latitude [" << lat
           << "], longitude [" << lon << "], elevation [" << ele << "]"
           << std::endl;

    // Set spherical coordinates
    ignition::msgs::SphericalCoordinates scReq;
    scReq.set_surface_model(ignition::msgs::SphericalCoordinates::EARTH_WGS84);
    scReq.set_latitude_deg(lat);
    scReq.set_longitude_deg(lon);
    scReq.set_elevation(ele);

    // Use zero heading so world is always aligned with lat / lon,
    // rotate vehicle instead.
    scReq.set_heading_deg(0.0);

    if (!this->node.Request(this->setSphericalCoordsService, scReq,
        &WorldCommPlugin::ServiceResponse, this))
    {
      ignerr << "Failed to request service [" << this->setSphericalCoordsService
             << "]" << std::endl;
    }
  }
  this->spawnCount++;

  // Create vehicle
  ignition::msgs::EntityFactory factoryReq;
  factoryReq.set_sdf(this->TethysSdfString(_msg));

  auto coords = factoryReq.mutable_spherical_coordinates();
  coords->set_surface_model(ignition::msgs::SphericalCoordinates::EARTH_WGS84);
  coords->set_latitude_deg(lat);
  coords->set_longitude_deg(lon);
  coords->set_elevation(ele);
  ignition::msgs::Set(factoryReq.mutable_pose()->mutable_orientation(),
      ignition::math::Quaterniond(
      _msg.initroll_(), _msg.initpitch_(), _msg.initheading_()));

  // RPH command is in NED
  // X == R: about N
  // Y == P: about E
  // Z == H: about D

  // Gazebo takes ENU
  // X == R: about E
  // Y == P: about N
  // Z == Y: about U

  auto rotENU = ignition::math::Quaterniond::EulerToQuaternion(
      // East: NED's pitch
      _msg.initpitch_(),
      // North: NED's roll
      _msg.initroll_(),
      // Up: NED's -yaw
      -_msg.initheading_());

  // The robot model is facing its own -X, so with zero ENU orientation it faces
  // West. We add an extra 90 degree yaw so zero means North, to conform with
  // NED.
  auto rotRobot = ignition::math::Quaterniond(0.0, 0.0, -IGN_PI * 0.5) * rotENU;

  ignition::msgs::Set(factoryReq.mutable_pose()->mutable_orientation(), rotRobot);

  // TODO(chapulina) Check what's up with all the errors
  if (!this->node.Request(this->createService, factoryReq,
      &WorldCommPlugin::ServiceResponse, this))
  {
    ignerr << "Failed to request service [" << this->createService
           << "]" << std::endl;
  }
}

/////////////////////////////////////////////////
std::string WorldCommPlugin::TethysSdfString(const lrauv_ignition_plugins::msgs::LRAUVInit &_msg)
{
  const std::string _id = _msg.id_().data();
  const std::string _acommsAddress = std::to_string(_msg.acommsaddress_());

  const std::string sdfStr = R"(
  <sdf version="1.9">
  <model name=")" + _id + R"(">
    <include merge="true">

      <!--
          Without any extra pose offset, the model is facing West.
          For the controller, zero orientation means the robot is facing North.
          So we need to rotate it.
          Note that this pose is expressed in ENU.
      <pose degrees="true">0 0 0  0 0 -90</pose>
      -->

      <!-- rename included model to avoid frame collisions -->
      <name>tethys_equipped</name>

      <uri>tethys_equipped</uri>

      <experimental:params>

        <sensor element_id="base_link::salinity_sensor" action="modify">
          <topic>/model/)" + _id + R"(/salinity</topic>
        </sensor>

        <sensor element_id="base_link::temperature_sensor" action="modify">
          <topic>/model/)" + _id + R"(/temperature</topic>
        </sensor>

        <sensor element_id="base_link::chlorophyll_sensor" action="modify">
          <topic>/model/)" + _id + R"(/chlorophyll</topic>
        </sensor>

        <sensor element_id="base_link::current_sensor" action="modify">
          <topic>/model/)" + _id + R"(/current</topic>
        </sensor>

        <sensor element_id="base_link::sparton_ahrs_m2_imu" action="modify">
          <topic>/)" + _id + R"(/ahrs/imu</topic>
        </sensor>

        <sensor element_id="base_link::sparton_ahrs_m2_magnetometer" action="modify">
          <topic>/)" + _id + R"(/ahrs/magnetometer</topic>
        </sensor>

        <plugin element_id="ignition::gazebo::systems::Thruster" action="modify">
          <namespace>)" + _id + R"(</namespace>
        </plugin>

        <plugin element_id="tethys::TethysCommPlugin" action="modify">
          <namespace>)" + _id + R"(</namespace>
          <command_topic>)" + _id + R"(/command_topic</command_topic>
          <state_topic>)" + _id + R"(/state_topic</state_topic>
        </plugin>

        <plugin element_id="ignition::gazebo::systems::BuoyancyEngine" action="modify">
          <namespace>)" + _id + R"(</namespace>
        </plugin>

        <plugin element_id="ignition::gazebo::systems::DetachableJoint" action="modify">
          <topic>/model/)" + _id + R"(/drop_weight</topic>
        </plugin>

        <plugin element_id="tethys::AcousticCommsPlugin" action="modify">
          <address>)" + _acommsAddress + R"(</address>
        </plugin>

        <plugin element_id="tethys::RangeBearingPlugin" action="modify">
          <address>)" + _acommsAddress + R"(</address>
          <namespace>)" + _id + R"(</namespace>
        </plugin>

      </experimental:params>
    </include>
  </model>
  </sdf>)";

  return sdfStr;
}

IGNITION_ADD_PLUGIN(
  tethys::WorldCommPlugin,
  ignition::gazebo::System,
  tethys::WorldCommPlugin::ISystemConfigure)
