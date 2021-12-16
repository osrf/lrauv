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
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/header.pb.h>
#include <ignition/msgs/time.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/TopicUtils.hh>

#include "lrauv_command.pb.h"
#include "lrauv_state.pb.h"

#include "TethysCommPlugin.hh"

// East-North-Up to North-East-Down
#define ENU_TO_NED ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, IGN_PI * 0.5)

// Transform model frame to FSK
// Tethys model is currently setup as:
// X: Back
// Y: Starboard (right)
// Z: Up
// The controller expects FSK:
// X: Forward
// Y: Starboard (right)
// Z: Keel (down)
// We should make our lives easier by aligning the model with FSK, see
// https://github.com/osrf/lrauv/issues/80
#define MODEL_TO_FSK ignition::math::Pose3d(0, 0, 0, 0, IGN_PI, 0)

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
    ignerr << "Latitude range is [-90, 90]. Received [" << _lat << "]"
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
}

void AddJointPosition(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  auto jointPosComp =
      _ecm.Component<ignition::gazebo::components::JointPosition>(_entity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
      _entity, ignition::gazebo::components::JointPosition());
  }
}

void AddJointVelocity(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  auto jointPosComp =
      _ecm.Component<ignition::gazebo::components::JointVelocity>(_entity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
      _entity, ignition::gazebo::components::JointVelocity());
  }
}

void AddWorldLinearVelocity(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldLinearVelocity());
  }
}

void TethysCommPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
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
  if (_sdf->HasElement("density"))
  {
    this->oceanDensity = _sdf->Get<double>("ocean_density");
  }

  // Initialize transport
  if (!this->node.Subscribe(this->commandTopic,
      &TethysCommPlugin::CommandCallback, this))
  {
    ignerr << "Error subscribing to topic " << "[" << this->commandTopic
      << "]. " << std::endl;
    return;
  }

  this->statePub =
    this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVState>(
    this->stateTopic);
  if (!this->statePub)
  {
    ignerr << "Error advertising topic [" << this->stateTopic << "]"
      << std::endl;
  }

  SetupControlTopics(ns);
  SetupEntities(_entity, _sdf, _ecm, _eventMgr);
}

void TethysCommPlugin::SetupControlTopics(const std::string &_ns)
{
  this->thrusterTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->thrusterTopic);
  this->thrusterPub =
    this->node.Advertise<ignition::msgs::Double>(this->thrusterTopic);
  if (!this->thrusterPub)
  {
    ignerr << "Error advertising topic [" << this->thrusterTopic << "]"
      << std::endl;
  }

  this->rudderTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->rudderTopic);
  this->rudderPub =
    this->node.Advertise<ignition::msgs::Double>(this->rudderTopic);
  if (!this->rudderPub)
  {
    ignerr << "Error advertising topic [" << this->rudderTopic << "]"
      << std::endl;
  }

  this->elevatorTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->elevatorTopic);
  this->elevatorPub =
    this->node.Advertise<ignition::msgs::Double>(this->elevatorTopic);
  if (!this->elevatorPub)
  {
    ignerr << "Error advertising topic [" << this->elevatorTopic << "]"
      << std::endl;
  }

  this->massShifterTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/joint/" + this->massShifterTopic);
  this->massShifterPub =
    this->node.Advertise<ignition::msgs::Double>(this->massShifterTopic);
  if (!this->massShifterPub)
  {
    ignerr << "Error advertising topic [" << this->massShifterTopic << "]"
      << std::endl;
  }

  // Publisher for command to buoyancy engine plugin
  this->buoyancyEngineCmdTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->buoyancyEngineCmdTopic);
  this->buoyancyEnginePub =
    this->node.Advertise<ignition::msgs::Double>(this->buoyancyEngineCmdTopic);
  if (!this->buoyancyEnginePub)
  {
    ignerr << "Error advertising topic [" << this->buoyancyEngineCmdTopic << "]"
      << std::endl;
  }

  // Subscribe to requests for buoyancy state
  this->buoyancyEngineStateTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->buoyancyEngineStateTopic);
  if (!this->node.Subscribe(this->buoyancyEngineStateTopic,
      &TethysCommPlugin::BuoyancyStateCallback, this))
  {
    ignerr << "Error subscribing to topic " << "["
      << this->buoyancyEngineStateTopic << "]. " << std::endl;
  }

  this->dropWeightTopic = ignition::transport::TopicUtils::AsValidTopic("/model/" +
    _ns + "/" + this->dropWeightTopic);
  this->dropWeightPub =
    this->node.Advertise<ignition::msgs::Empty>(this->dropWeightTopic);
  if(!this->dropWeightPub)
  {
    ignerr << "Error advertising topic [" << this->dropWeightTopic << "]"
      << std::endl;
  }

  // Subscribe to sensor data
  this->salinityTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->salinityTopic);
  if (!this->node.Subscribe(this->salinityTopic,
      &TethysCommPlugin::SalinityCallback, this))
  {
    ignerr << "Error subscribing to topic " << "["
      << this->salinityTopic << "]. " << std::endl;
  }

  this->temperatureTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->temperatureTopic);
  if (!this->node.Subscribe(this->temperatureTopic,
      &TethysCommPlugin::TemperatureCallback, this))
  {
    ignerr << "Error subscribing to topic " << "["
      << this->temperatureTopic << "]. " << std::endl;
  }

  this->chlorophyllTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->chlorophyllTopic);
  if (!this->node.Subscribe(this->chlorophyllTopic,
      &TethysCommPlugin::ChlorophyllCallback, this))
  {
    ignerr << "Error subscribing to topic " << "["
      << this->chlorophyllTopic << "]. " << std::endl;
  }

  this->currentTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + _ns + "/" + this->currentTopic);
  if (!this->node.Subscribe(this->currentTopic,
      &TethysCommPlugin::CurrentCallback, this))
  {
    ignerr << "Error subscribing to topic " << "["
      << this->currentTopic << "]. " << std::endl;
  }
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

  auto model = ignition::gazebo::Model(_entity);

  this->modelLink = model.LinkByName(_ecm, this->baseLinkName);

  this->thrusterLink = model.LinkByName(_ecm, this->thrusterLinkName);
  this->thrusterJoint = model.JointByName(_ecm, thrusterJointName);

  this->rudderJoint = model.JointByName(_ecm, this->rudderJointName);
  this->elevatorJoint = model.JointByName(_ecm, this->elevatorJointName);
  this->massShifterJoint = model.JointByName(_ecm, this->massShifterJointName);

  AddAngularVelocityComponent(this->thrusterLink, _ecm);
  AddWorldPose(this->modelLink, _ecm);
  AddJointPosition(this->rudderJoint, _ecm);
  AddJointPosition(this->elevatorJoint, _ecm);
  AddJointPosition(this->massShifterJoint, _ecm);
  AddJointVelocity(this->thrusterJoint, _ecm);
  AddWorldLinearVelocity(this->modelLink, _ecm);
}

void TethysCommPlugin::CommandCallback(
  const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg)
{
  if (this->debugPrintout)
  {
    igndbg << "[" << this->ns << "] Received command: " << std::endl
      << _msg.DebugString() << std::endl;
  }

  // Rudder
  ignition::msgs::Double rudderAngMsg;
  rudderAngMsg.set_data(_msg.rudderangleaction_());
  this->rudderPub.Publish(rudderAngMsg);

  // Elevator
  ignition::msgs::Double elevatorAngMsg;
  elevatorAngMsg.set_data(_msg.elevatorangleaction_());
  this->elevatorPub.Publish(elevatorAngMsg);

  // Thruster
  ignition::msgs::Double thrusterMsg;
  // TODO(arjo):
  // Conversion from angular velocity to force b/c thruster plugin takes force
  // Maybe we should change that?
  // https://github.com/osrf/lrauv/issues/75
  auto angVel = _msg.propomegaaction_();

  // force = thrust_coefficient * fluid_density * omega ^ 2 *
  //         propeller_diameter ^ 4
  // These values are defined in the model's Thruster plugin's SDF
  auto force = 0.004422 * 1000 * pow(0.2, 4) * angVel * angVel;
  if (angVel < 0)
  {
    force *=-1;
  }
  thrusterMsg.set_data(force);
  this->thrusterPub.Publish(thrusterMsg);

  // Mass shifter
  ignition::msgs::Double massShifterMsg;
  massShifterMsg.set_data(_msg.masspositionaction_());
  this->massShifterPub.Publish(massShifterMsg);

  // Buoyancy Engine
  ignition::msgs::Double buoyancyEngineMsg;
  buoyancyEngineMsg.set_data(_msg.buoyancyaction_());
  this->buoyancyEnginePub.Publish(buoyancyEngineMsg);

  // Drop weight
  auto dropweight = _msg.dropweightstate_();
  // Indicator is true (1) for dropweight OK in place, false (0) for dropped
  if (!dropweight)
  {
    ignition::msgs::Empty dropWeightCmd;
    this->dropWeightPub.Publish(dropWeightCmd);
  }
}

void TethysCommPlugin::BuoyancyStateCallback(
  const ignition::msgs::Double &_msg)
{
  this->buoyancyBladderVolume = _msg.data();
}

void TethysCommPlugin::SalinityCallback(
  const ignition::msgs::Float &_msg)
{
  this->latestSalinity = _msg.data();
}

void TethysCommPlugin::TemperatureCallback(
  const ignition::msgs::Double &_msg)
{
  this->latestTemperature.SetCelsius(_msg.data());
}

void TethysCommPlugin::ChlorophyllCallback(
  const ignition::msgs::Float &_msg)
{
  this->latestChlorophyll = _msg.data();
}

void TethysCommPlugin::CurrentCallback(
  const ignition::msgs::Vector3d &_msg)
{
  this->latestCurrent = ignition::msgs::Convert(_msg);
}

void TethysCommPlugin::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  lrauv_ignition_plugins::msgs::LRAUVState stateMsg;

  ///////////////////////////////////
  // Header
  stateMsg.mutable_header()->mutable_stamp()->set_sec(
    std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
  stateMsg.mutable_header()->mutable_stamp()->set_nsec(
    int(std::chrono::duration_cast<std::chrono::nanoseconds>(
    _info.simTime).count()) - stateMsg.header().stamp().sec() * 1000000000);

  ///////////////////////////////////
  // Actuators

  // TODO(anyone) Maybe angular velocity should come from ThrusterPlugin
  // Propeller angular velocity
  auto propAngVelComp =
    _ecm.Component<ignition::gazebo::components::JointVelocity>(thrusterJoint);
  if (propAngVelComp->Data().size() != 1)
  {
    ignerr << "Propeller joint has wrong size\n";
    return;
  }
  stateMsg.set_propomega_(propAngVelComp->Data()[0]);

  // Rudder joint position
  auto rudderPosComp =
    _ecm.Component<ignition::gazebo::components::JointPosition>(rudderJoint);
  if (rudderPosComp->Data().size() != 1)
  {
    ignerr << "Rudder joint has wrong size\n";
    return;
  }
  stateMsg.set_rudderangle_(rudderPosComp->Data()[0]);

  // Elevator joint position
  auto elevatorPosComp =
    _ecm.Component<ignition::gazebo::components::JointPosition>(elevatorJoint);
  if (elevatorPosComp->Data().size() != 1)
  {
    ignerr << "Elavator joint has wrong size\n";
    return;
  }
  stateMsg.set_elevatorangle_(elevatorPosComp->Data()[0]);

  // Mass shifter joint position
  auto massShifterPosComp =
    _ecm.Component<ignition::gazebo::components::JointPosition>(
    massShifterJoint);
  if (massShifterPosComp->Data().size() != 1)
  {
    ignerr << "Mass shifter joint component has the wrong size ("
      << massShifterPosComp->Data().size() << "), expected 1\n";
    return;
  }
  stateMsg.set_massposition_(massShifterPosComp->Data()[0]);

  // Buoyancy position
  stateMsg.set_buoyancyposition_(this->buoyancyBladderVolume);

  ///////////////////////////////////
  // Position

  // Gazebo is using ENU, controller expects NED
  auto modelPoseENU = ignition::gazebo::worldPose(this->modelLink, _ecm);
  auto modelPoseNEDRot = ENU_TO_NED * modelPoseENU;
  ignition::math::Pose3d modelPoseNED{
      modelPoseENU.Y(), modelPoseENU.X(), -modelPoseENU.Z(),
      modelPoseENU.Pitch(), modelPoseENU.Roll(), -modelPoseENU.Yaw()};

  stateMsg.set_depth_(-modelPoseENU.Pos().Z());

//ignerr << modelPoseNEDRot << " --- " << modelPoseNED << std::endl;

  ignition::msgs::Set(stateMsg.mutable_pos_(), modelPoseNED.Pos());
  ignition::msgs::Set(stateMsg.mutable_rph_(), modelPoseNED.Rot().Euler());
  ignition::msgs::Set(stateMsg.mutable_posrph_(), modelPoseNED.Rot().Euler());

  auto latlon = ignition::gazebo::sphericalCoordinates(this->modelLink, _ecm);
  if (latlon)
  {
    stateMsg.set_latitudedeg_(latlon.value().X());
    stateMsg.set_longitudedeg_(latlon.value().Y());
  }

  ///////////////////////////////////
  // Velocity

  auto linVelComp =
    _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(
    this->modelLink);
  if (nullptr != linVelComp)
  {
    auto linVelENU = linVelComp->Data();
    stateMsg.set_speed_(linVelENU.Length());

    // World frame in NED
    auto linVelNED = ENU_TO_NED.Rot() * linVelENU;
    ignition::msgs::Set(stateMsg.mutable_posdot_(), linVelNED);

    // Vehicle frame in FSK
    auto linVelFSK = MODEL_TO_FSK.Rot() * linVelENU;
    ignition::msgs::Set(stateMsg.mutable_rateuvw_(), linVelFSK);
  }

  auto angVelComp =
    _ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(
    this->modelLink);
  if (nullptr != angVelComp)
  {
    auto angVelENU = angVelComp->Data();

    // Vehicle frame in FSK
    auto angVelFSK = MODEL_TO_FSK.Rot() * angVelENU;
    ignition::msgs::Set(stateMsg.mutable_ratepqr_(), angVelFSK);
  }

  ///////////////////////////////////
  // Sensor data
  stateMsg.set_salinity_(this->latestSalinity);
  stateMsg.set_temperature_(this->latestTemperature.Celsius());
  stateMsg.add_values_(this->latestChlorophyll);

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
    igndbg << "[" << this->ns << "] Published state to " << this->stateTopic
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
      << "\tPressure (Pa): " << stateMsg.values_(1) << std::endl;

    this->prevPubPrintTime = _info.simTime;
  }
}

IGNITION_ADD_PLUGIN(
  tethys::TethysCommPlugin,
  ignition::gazebo::System,
  tethys::TethysCommPlugin::ISystemConfigure,
  tethys::TethysCommPlugin::ISystemPostUpdate)
