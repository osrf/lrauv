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

#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/TopicUtils.hh>

#include "lrauv_gazebo_plugins/lrauv_command.pb.h"
#include "lrauv_gazebo_plugins/lrauv_state.pb.h"

#include "TethysCommPlugin.hh"

using namespace tethys;

/// \brief Calculates water pressure based on depth and latitude.
/// Borrowed from MBARI's codebase: AuvMath::OceanPressure, which implements
/// Peter M. Saunders, Practical Conversion of Pressure to Depth, Journal of
/// Physical Oceanography, 11(4), 573--574, 1981. DOI
/// https://doi.org/10.1175/1520-0485(1981)011%3C0573:PCOPTD%3E2.0.CO;2
/// \param[in] _depth Depth in meters, larger values are deeper.
/// \param[in] _lat Latitude in degrees.
/// \return Pressure in Pa, or -1 in case of error.
double pressureFromDepthLatitude(double _depth, double _lat)
{
  // Negative check with tolerance
  if (_depth < -1E-5)
  {
    // This happens often when the vehicle is near the surface, so we don't
    // spam the console with error messages.
    return -1.0;
  }
  if (_lat < -90 || _lat > 90)
  {
    gzerr << "Latitude range is [-90, 90]. Received [" << _lat << "]"
           << std::endl;
    return -1.0;
  }

  const double G0 = 9.780318;
  const double G1 = 5.2788E-3;

  double x = sin(_lat);
  x *= x;

  double gLatitude =  G0 * (1 + G1 * x);

  double kDepthLatitude = (gLatitude - 2E-5 * _depth)
                        / (9.80612 - 2E-5 * _depth);

  double depth2 = _depth * _depth;

  double hDepth45 = 1.00818E-2 * _depth
                    + 2.465E-8 * depth2
                    - 1.25E-13 * depth2 * _depth
                    + 2.8E-19 * depth2 * depth2;

  double hDepthLatitude = hDepth45 * kDepthLatitude;

  double thyh0Depth = _depth / (_depth + 100) / 100 + 6.2E-6 * _depth;

  double pDepthLatitude = hDepthLatitude - thyh0Depth;

  return pDepthLatitude * 1E6;
}

void AddAngularVelocityComponent(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<gz::sim::components::AngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      gz::sim::components::AngularVelocity());
  }

  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<gz::sim::components::WorldAngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      gz::sim::components::WorldAngularVelocity());
  }
}

void AddWorldPose (
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<gz::sim::components::WorldPose>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      gz::sim::components::WorldPose());
  }
}

void AddJointPosition(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  auto jointPosComp =
      _ecm.Component<gz::sim::components::JointPosition>(_entity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
      _entity, gz::sim::components::JointPosition());
  }
}

void AddJointVelocity(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  auto jointPosComp =
      _ecm.Component<gz::sim::components::JointVelocity>(_entity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
      _entity, gz::sim::components::JointVelocity());
  }
}

void AddWorldLinearVelocity(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<gz::sim::components::WorldLinearVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      gz::sim::components::WorldLinearVelocity());
  }
}

/// \brief Convert a vector from "SFU" frame to FSK.
/// \param[in] _sfu Vector in model frame, which is oriented "SFU":
///     X: Starboard / Right
///     Y: Forward
///     Z: Up
/// \return Vector in FSK, which is:
///     X: Forward
///     Y: Starboard / Right
///     Z: Up
gz::math::Vector3d SFUToFSK(const gz::math::Vector3d &_sfu)
{
  return {_sfu.Y(), _sfu.X(), -_sfu.Z()};
}

/// \brief Convert a pose in ENU to NED.
/// \param[in] _enu A pose in ENU (Gazebo's world frame)
/// \return A pose in NED (what the controller expects)
gz::math::Pose3d ENUToNED(const gz::math::Pose3d &_enu)
{
  return {_enu.Y(), _enu.X(), -_enu.Z(),
          _enu.Pitch(), _enu.Roll(), -_enu.Yaw()};
}

// Convert a vector in ENU to NED.
gz::math::Vector3d ENUToNED(const gz::math::Vector3d &_enu)
{
  return {_enu.Y(), _enu.X(), -_enu.Z()};
}

void TethysCommPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_eventMgr)
{
  // Get namespace
  if (_sdf->HasElement("namespace"))
  {
    this->ns = _sdf->Get<std::string>("namespace");
  }
  else
  {
    this->ns = "tethys";
  }

  // Parse SDF parameters
  if (_sdf->HasElement("command_topic"))
  {
    this->commandTopic = _sdf->Get<std::string>("command_topic");
  }
  if (_sdf->HasElement("state_topic"))
  {
    this->stateTopic = _sdf->Get<std::string>("state_topic");
  }
  if (_sdf->HasElement("debug_printout"))
  {
    this->debugPrintout = _sdf->Get<bool>("debug_printout");
  }
  if (_sdf->HasElement("ocean_density"))
  {
    this->oceanDensity = _sdf->Get<double>("ocean_density");
  }

  // Initialize transport
  if (!this->node.Subscribe(this->commandTopic,
      &TethysCommPlugin::CommandCallback, this))
  {
    gzerr << "Error subscribing to topic " << "[" << this->commandTopic
      << "]. " << std::endl;
    return;
  }

  this->statePub =
    this->node.Advertise<lrauv_gazebo_plugins::msgs::LRAUVState>(
    this->stateTopic);
  if (!this->statePub)
  {
    gzerr << "Error advertising topic [" << this->stateTopic << "]"
      << std::endl;
  }

  std::string navSatTopic = this->ns + "/navsat";
  this->navSatPub =
    this->node.Advertise<gz::msgs::NavSat>(navSatTopic);
  if (!this->navSatPub)
  {
    gzerr << "Error advertising topic [" << navSatTopic << "]" << std::endl;
  }

  SetupControlTopics(ns);
  SetupEntities(_entity, _sdf, _ecm, _eventMgr);
}

void TethysCommPlugin::SetupControlTopics(const std::string &_ns)
{
  this->thrusterTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->thrusterTopic);
  this->thrusterPub =
    this->node.Advertise<gz::msgs::Double>(this->thrusterTopic);
  if (!this->thrusterPub)
  {
    gzerr << "Error advertising topic [" << this->thrusterTopic << "]"
      << std::endl;
  }

  this->rudderTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->rudderTopic);
  this->rudderPub =
    this->node.Advertise<gz::msgs::Double>(this->rudderTopic);
  if (!this->rudderPub)
  {
    gzerr << "Error advertising topic [" << this->rudderTopic << "]"
      << std::endl;
  }

  this->elevatorTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->elevatorTopic);
  this->elevatorPub =
    this->node.Advertise<gz::msgs::Double>(this->elevatorTopic);
  if (!this->elevatorPub)
  {
    gzerr << "Error advertising topic [" << this->elevatorTopic << "]"
      << std::endl;
  }

  this->massShifterTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->massShifterTopic);
  this->massShifterPub =
    this->node.Advertise<gz::msgs::Double>(this->massShifterTopic);
  if (!this->massShifterPub)
  {
    gzerr << "Error advertising topic [" << this->massShifterTopic << "]"
      << std::endl;
  }

  // Publisher for command to buoyancy engine plugin
  this->buoyancyEngineCmdTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->buoyancyEngineCmdTopic);
  this->buoyancyEnginePub =
    this->node.Advertise<gz::msgs::Double>(this->buoyancyEngineCmdTopic);
  if (!this->buoyancyEnginePub)
  {
    gzerr << "Error advertising topic [" << this->buoyancyEngineCmdTopic << "]"
      << std::endl;
  }

  // Subscribe to requests for buoyancy state
  this->buoyancyEngineStateTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->buoyancyEngineStateTopic);
  if (!this->node.Subscribe(this->buoyancyEngineStateTopic,
      &TethysCommPlugin::BuoyancyStateCallback, this))
  {
    gzerr << "Error subscribing to topic " << "["
      << this->buoyancyEngineStateTopic << "]. " << std::endl;
  }

  this->dropWeightTopic = gz::transport::TopicUtils::AsValidTopic("/model/" +
    _ns + "/" + this->dropWeightTopic);
  this->dropWeightPub =
    this->node.Advertise<gz::msgs::Empty>(this->dropWeightTopic);
  if(!this->dropWeightPub)
  {
    gzerr << "Error advertising topic [" << this->dropWeightTopic << "]"
      << std::endl;
  }

  // Subscribe to sensor data
  this->salinityTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->salinityTopic);
  if (!this->node.Subscribe(this->salinityTopic,
      &TethysCommPlugin::SalinityCallback, this))
  {
    gzerr << "Error subscribing to topic " << "["
      << this->salinityTopic << "]. " << std::endl;
  }

  this->temperatureTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->temperatureTopic);
  if (!this->node.Subscribe(this->temperatureTopic,
      &TethysCommPlugin::TemperatureCallback, this))
  {
    gzerr << "Error subscribing to topic " << "["
      << this->temperatureTopic << "]. " << std::endl;
  }

  this->batteryTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->batteryTopic);
  if (!this->node.Subscribe(this->batteryTopic,
      &TethysCommPlugin::BatteryCallback, this))
  {
    gzerr << "Error subscribing to topic " << "["
      << this->batteryTopic << "]. " << std::endl;
  }

  this->chlorophyllTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->chlorophyllTopic);
  if (!this->node.Subscribe(this->chlorophyllTopic,
      &TethysCommPlugin::ChlorophyllCallback, this))
  {
    gzerr << "Error subscribing to topic " << "["
      << this->chlorophyllTopic << "]. " << std::endl;
  }

  this->currentTopic = gz::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->currentTopic);
  if (!this->node.Subscribe(this->currentTopic,
      &TethysCommPlugin::CurrentCallback, this))
  {
    gzerr << "Error subscribing to topic " << "["
      << this->currentTopic << "]. " << std::endl;
  }
}

void TethysCommPlugin::SetupEntities(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_eventMgr)
{
  if (_sdf->HasElement("model_link"))
  {
    this->baseLinkName = _sdf->Get<std::string>("model_link");
  }

  if (_sdf->HasElement("propeller_link"))
  {
    this->thrusterLinkName = _sdf->Get<std::string>("propeller_link");
  }

  if (_sdf->HasElement("rudder_joint"))
  {
    this->rudderJointName = _sdf->Get<std::string>("rudder_joint");
  }

  if (_sdf->HasElement("elavator_joint"))
  {
    this->elevatorJointName = _sdf->Get<std::string>("elavator_joint");
  }

  if (_sdf->HasElement("mass_shifter_joint"))
  {
    this->massShifterJointName = _sdf->Get<std::string>("mass_shifter_joint");
  }

  this->modelEntity = _entity;

  auto model = gz::sim::Model(_entity);

  this->baseLink = model.LinkByName(_ecm, this->baseLinkName);

  this->thrusterLink = model.LinkByName(_ecm, this->thrusterLinkName);
  this->thrusterJoint = model.JointByName(_ecm, thrusterJointName);

  this->rudderJoint = model.JointByName(_ecm, this->rudderJointName);
  this->elevatorJoint = model.JointByName(_ecm, this->elevatorJointName);
  this->massShifterJoint = model.JointByName(_ecm, this->massShifterJointName);

  AddAngularVelocityComponent(this->thrusterLink, _ecm);
  AddWorldPose(this->baseLink, _ecm);
  AddJointPosition(this->rudderJoint, _ecm);
  AddJointPosition(this->elevatorJoint, _ecm);
  AddJointPosition(this->massShifterJoint, _ecm);
  AddJointVelocity(this->thrusterJoint, _ecm);
  AddWorldLinearVelocity(this->baseLink, _ecm);

  gz::sim::enableComponent
      <gz::sim::components::WorldAngularVelocity>(_ecm,
      this->baseLink);
}

void TethysCommPlugin::CommandCallback(
  const lrauv_gazebo_plugins::msgs::LRAUVCommand &_msg)
{
  if (this->debugPrintout)
  {
    gzdbg << "[" << this->ns << "] Received command: " << std::endl
      << _msg.DebugString() << std::endl;
  }

  // Rudder
  gz::msgs::Double rudderAngMsg;
  rudderAngMsg.set_data(_msg.rudderangleaction_());
  this->rudderPub.Publish(rudderAngMsg);

  // Elevator
  gz::msgs::Double elevatorAngMsg;
  elevatorAngMsg.set_data(_msg.elevatorangleaction_());
  this->elevatorPub.Publish(elevatorAngMsg);

  // Thruster
  gz::msgs::Double thrusterMsg;
  auto angVel = _msg.propomegaaction_();
  thrusterMsg.set_data(angVel);
  this->thrusterPub.Publish(thrusterMsg);

  // Mass shifter
  gz::msgs::Double massShifterMsg;
  massShifterMsg.set_data(_msg.masspositionaction_());
  this->massShifterPub.Publish(massShifterMsg);

  // Buoyancy Engine
  gz::msgs::Double buoyancyEngineMsg;
  buoyancyEngineMsg.set_data(_msg.buoyancyaction_());
  this->buoyancyEnginePub.Publish(buoyancyEngineMsg);

  // Drop weight
  auto dropweight = _msg.dropweightstate_();
  // Indicator is true (1) for dropweight OK in place, false (0) for dropped
  if (!dropweight)
  {
    gz::msgs::Empty dropWeightCmd;
    this->dropWeightPub.Publish(dropWeightCmd);
  }
}

void TethysCommPlugin::BuoyancyStateCallback(
  const gz::msgs::Double &_msg)
{
  this->buoyancyBladderVolume = _msg.data();
}

void TethysCommPlugin::SalinityCallback(
  const gz::msgs::Float &_msg)
{
  this->latestSalinity = _msg.data();
}

void TethysCommPlugin::TemperatureCallback(
  const gz::msgs::Double &_msg)
{
  this->latestTemperature.SetCelsius(_msg.data());
}

void TethysCommPlugin::BatteryCallback(
  const gz::msgs::BatteryState &_msg)
{
  this->latestBatteryVoltage = _msg.voltage();
  this->latestBatteryCurrent = _msg.current();
  this->latestBatteryCharge = _msg.charge();
  this->latestBatteryPercentage = _msg.percentage();
}

void TethysCommPlugin::ChlorophyllCallback(
  const gz::msgs::Float &_msg)
{
  this->latestChlorophyll = _msg.data();
}

void TethysCommPlugin::CurrentCallback(
  const gz::msgs::Vector3d &_msg)
{
  this->latestCurrent = gz::msgs::Convert(_msg);
}

void TethysCommPlugin::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Publish state
  lrauv_gazebo_plugins::msgs::LRAUVState stateMsg;

  ///////////////////////////////////
  // Header
  stateMsg.mutable_header()->mutable_stamp()->set_sec(
    std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
  stateMsg.mutable_header()->mutable_stamp()->set_nsec(
    int(std::chrono::duration_cast<std::chrono::nanoseconds>(
    _info.simTime).count()) - stateMsg.header().stamp().sec() * 1000000000);

  ///////////////////////////////////
  // Actuators
  auto propAngVelComp =
    _ecm.Component<gz::sim::components::JointVelocity>(thrusterJoint);
  if (propAngVelComp->Data().size() != 1)
  {
    gzerr << "Propeller joint has wrong size\n";
    return;
  }
  stateMsg.set_propomega_(propAngVelComp->Data()[0]);

  // Rudder joint position
  auto rudderPosComp =
    _ecm.Component<gz::sim::components::JointPosition>(rudderJoint);
  if (rudderPosComp->Data().size() != 1)
  {
    gzerr << "Rudder joint has wrong size\n";
    return;
  }
  stateMsg.set_rudderangle_(rudderPosComp->Data()[0]);

  // Elevator joint position
  auto elevatorPosComp =
    _ecm.Component<gz::sim::components::JointPosition>(elevatorJoint);
  if (elevatorPosComp->Data().size() != 1)
  {
    gzerr << "Elavator joint has wrong size\n";
    return;
  }
  stateMsg.set_elevatorangle_(elevatorPosComp->Data()[0]);

  // Mass shifter joint position
  auto massShifterPosComp =
    _ecm.Component<gz::sim::components::JointPosition>(
    massShifterJoint);
  if (massShifterPosComp->Data().size() != 1)
  {
    gzerr << "Mass shifter joint component has the wrong size ("
      << massShifterPosComp->Data().size() << "), expected 1\n";
    return;
  }
  stateMsg.set_massposition_(massShifterPosComp->Data()[0]);

  // Buoyancy position
  stateMsg.set_buoyancyposition_(this->buoyancyBladderVolume);

  ///////////////////////////////////
  // Position

  // Gazebo is using ENU, controller expects NED
  auto modelPoseENU = gz::sim::worldPose(this->modelEntity, _ecm);
  auto modelPoseNED = ENUToNED(modelPoseENU);

  stateMsg.set_depth_(-modelPoseENU.Pos().Z());

  gz::msgs::Set(stateMsg.mutable_pos_(), modelPoseNED.Pos());
  gz::msgs::Set(stateMsg.mutable_rph_(), modelPoseNED.Rot().Euler());
  gz::msgs::Set(stateMsg.mutable_posrph_(), modelPoseNED.Rot().Euler());

  auto latlon = gz::sim::sphericalCoordinates(this->modelEntity, _ecm);
  if (latlon)
  {
    stateMsg.set_latitudedeg_(latlon.value().X());
    stateMsg.set_longitudedeg_(latlon.value().Y());

    // Publish NavSat message to see position on NavSat GUI map
    gz::msgs::NavSat navSatMsg;
    navSatMsg.set_latitude_deg(latlon.value().X());
    navSatMsg.set_longitude_deg(latlon.value().Y());
    this->navSatPub.Publish(navSatMsg);
  }

  ///////////////////////////////////
  // Velocity

  // Speed
  auto linVelComp =
    _ecm.Component<gz::sim::components::WorldLinearVelocity>(
    this->baseLink);
  auto linVelENU = linVelComp->Data();
  stateMsg.set_speed_(linVelENU.Length());

  // Velocity in NED world frame
  auto linVelNED = ENUToNED(linVelENU);
  gz::msgs::Set(stateMsg.mutable_posdot_(), linVelNED);

  // Water velocity in FSK vehicle frame
  // TODO(arjo): include currents in water velocity?
  // World frame to local model frame
  auto linVelSFU = modelPoseENU.Rot().Inverse() * linVelENU;

  // Model frame is oriented "SFU", convert it to FSK
  auto linVelFSK = SFUToFSK(linVelSFU);
  gz::msgs::Set(stateMsg.mutable_rateuvw_(), linVelFSK);

  // Angular velocity in FSK
  auto angVelComp =
    _ecm.Component<gz::sim::components::WorldAngularVelocity>(
    this->baseLink);
  auto angVelENU = angVelComp->Data();

  // World frame to local model frame
  auto angVelSFU = modelPoseENU.Rot().Inverse() * angVelENU;

  // Model frame is oriented "SFU", convert it to FSK
  auto angVelFSK = SFUToFSK(angVelSFU);
  gz::msgs::Set(stateMsg.mutable_ratepqr_(), angVelFSK);

  // Sensor data
  stateMsg.set_salinity_(this->latestSalinity);
  stateMsg.set_temperature_(this->latestTemperature.Celsius());
  stateMsg.add_values_(this->latestChlorophyll);

  // Battery data
  stateMsg.set_batteryvoltage_(this->latestBatteryVoltage);
  stateMsg.set_batterycurrent_(this->latestBatteryCurrent);
  stateMsg.set_batterycharge_(this->latestBatteryCharge);
  stateMsg.set_batterypercentage_(this->latestBatteryPercentage);

  // Set Ocean Density
  stateMsg.set_density_(this->oceanDensity);

  double pressure = 0.0;
  if (latlon)
  {
    auto calcPressure = pressureFromDepthLatitude(-modelPoseENU.Pos().Z(),
      latlon.value().X());
    if (calcPressure >= 0)
      pressure = calcPressure;
  }

  stateMsg.add_values_(pressure);

  stateMsg.set_eastcurrent_(this->latestCurrent.X());
  stateMsg.set_northcurrent_(this->latestCurrent.Y());
  // Not populating vertCurrent because we're not getting it from the science
  // data

  this->statePub.Publish(stateMsg);

  if (this->debugPrintout &&
    _info.simTime - this->prevPubPrintTime > std::chrono::milliseconds(1000))
  {
    gzdbg << "[" << this->ns << "] Published state to " << this->stateTopic
      << " at time: " << stateMsg.header().stamp().sec()
      << "." << stateMsg.header().stamp().nsec() << std::endl
      << "\tLat / lon (deg): " << stateMsg.latitudedeg_() << " / "
                               << stateMsg.longitudedeg_() << std::endl
      << "\tpropOmega: " << stateMsg.propomega_() << std::endl
      << "\tSpeed: " << stateMsg.speed_() << std::endl
      << "\tElevator angle: " << stateMsg.elevatorangle_() << std::endl
      << "\tRudder angle: " << stateMsg.rudderangle_() << std::endl
      << "\tMass shifter (m): " << stateMsg.massposition_() << std::endl
      << "\tVBS volume (m^3): " << stateMsg.buoyancyposition_() << std::endl
      << "\tPitch angle (deg): "
        << stateMsg.rph_().y() * 180 / M_PI << std::endl
      << "\tCurrent (ENU, m/s): "
        << stateMsg.eastcurrent_() << ", "
        << stateMsg.northcurrent_() << ", "
        << stateMsg.vertcurrent_() << std::endl
      << "\tTemperature (C): " << stateMsg.temperature_() << std::endl
      << "\tSalinity (PSU): " << stateMsg.salinity_() << std::endl
      << "\tChlorophyll (ug/L): " << stateMsg.values_(0) << std::endl
      << "\tPressure (Pa): " << stateMsg.values_(1) << std::endl
      << "\tBattery Voltage (V): " << stateMsg.batteryvoltage_() << std::endl
      << "\tBattery Current (A): " << stateMsg.batterycurrent_() << std::endl
      << "\tBattery Charge (Ah): " << stateMsg.batterycharge_() << std::endl
      << "\tBattery Percentage (unitless): " << stateMsg.batterypercentage_() << std::endl;

    this->prevPubPrintTime = _info.simTime;
  }
}

GZ_ADD_PLUGIN(
  tethys::TethysCommPlugin,
  gz::sim::System,
  tethys::TethysCommPlugin::ISystemConfigure,
  tethys::TethysCommPlugin::ISystemPostUpdate)
