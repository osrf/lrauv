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

#ifndef TETHYS_COMM_PLUGIN_H_
#define TETHYS_COMM_PLUGIN_H_

#include <chrono>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/math/Temperature.hh>
#include <ignition/transport/Node.hh>

#include "lrauv_command.pb.h"

namespace tethys
{
  class TethysCommPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
    // Documentation inherited
    public: void Configure(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// Callback function for command from LRAUV Main Vehicle Application
    /// \param[in] _msg Command message
    public: void CommandCallback(
                const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg);

    /// Callback function for buoyancy bladder state
    /// \param[in] _msg Bladder volume
    public: void BuoyancyStateCallback(
                const ignition::msgs::Double &_msg);

    /// Callback function for salinity sensor data.
    /// \param[in] _msg Sensor data
    public: void SalinityCallback(
                const ignition::msgs::Float &_msg);

    /// Callback function for temperature sensor data.
    /// \param[in] _msg Sensor data
    public: void TemperatureCallback(
                const ignition::msgs::Double &_msg);

    /// Callback function for chlorophyll sensor data.
    /// \param[in] _msg Sensor data
    public: void ChlorophyllCallback(
                const ignition::msgs::Float &_msg);

    /// Callback function for current sensor data.
    /// \param[in] _msg Sensor data
    public: void CurrentCallback(
                const ignition::msgs::Vector3d &_msg);

    /// Parse SDF parameters and create components
    private: void SetupEntities(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr);

    /// Set up control message topics
    /// \param[in] _ns Namespace to prepend to topic names
    private: void SetupControlTopics(const std::string &_ns);

    /// Enable debug printout
    private: bool debugPrintout = false;

    /// Namespace for topics.
    private: std::string ns{""};

    /// Topic on which robot commands will be received
    private: std::string commandTopic{"command_topic"};

    /// Topic on which robot state will be published
    private: std::string stateTopic{"state_topic"};

    /// Topic to publish to for thruster
    private: std::string thrusterTopic
      {"propeller_joint/cmd_pos"};

    /// Topic to publish to for rudder
    private: std::string rudderTopic
      {"vertical_fins_joint/0/cmd_pos"};

    /// Topic to publish to for elevator
    private: std::string elevatorTopic
      {"horizontal_fins_joint/0/cmd_pos"};

    /// Topic to publish to for mass shifter
    private: std::string massShifterTopic
      {"battery_joint/0/cmd_pos"};

    /// Topic to write buoyancy engine's set point to
    private: std::string buoyancyEngineCmdTopic
      {"buoyancy_engine"};

    /// Topic to read buoyancy engine state from
    private: std::string buoyancyEngineStateTopic
      {"buoyancy_engine/current_volume"};

    /// Topic to publish for dropweight
    private: std::string dropWeightTopic
      {"drop_weight"};

    /// Topic to subscribe to salinity data
    private: std::string salinityTopic
      {"salinity"};

    /// Topic to subscribe to temperature data
    private: std::string temperatureTopic
      {"temperature"};

    /// Topic to subscribe to chlorophyll data
    private: std::string chlorophyllTopic
      {"chlorophyll"};

    /// Topic to subscribe to current data
    private: std::string currentTopic
      {"current"};

    /// Model name
    private: std::string baseLinkName{"base_link"};

    /// Propeller link name
    private: std::string thrusterLinkName{"propeller"};

    /// Propeller joint name
    private: std::string thrusterJointName{"propeller_joint"};

    /// Rudder joint name
    private: std::string rudderJointName{"vertical_fins_joint"};

    /// Elevator joint name
    private: std::string elevatorJointName{"horizontal_fins_joint"};

    /// Mass shifter joint name
    private: std::string massShifterJointName{"battery_joint"};

    /// TODO(mabelzhang) Remove when stable. Temporary counter for state
    /// message sanity check
    private: int counter = 0;

    /// Buoyancy bladder size in cc
    private: double buoyancyBladderVolume = 300;

    /// Latest salinity data received from sensor. NaN if not received.
    private: float latestSalinity{std::nanf("")};

    /// Latest temperature data received from sensor. NaN if not received.
    private: ignition::math::Temperature latestTemperature{std::nanf("")};

    /// Latest chlorophyll data received from sensor. NaN if not received.
    private: float latestChlorophyll{std::nanf("")};

    /// Ocean Density in kg / m ^ 3
    private: double oceanDensity{1000};

    /// Latest current data received from sensor. NaN if not received.
    private: ignition::math::Vector3d latestCurrent
        {std::nan(""), std::nan(""), std::nan("")};

    /// TODO(mabelzhang) Remove when stable. Temporary timers for state message
    /// sanity check
    private: std::chrono::steady_clock::duration prevPubPrintTime =
      std::chrono::steady_clock::duration::zero();

    /// Transport node for message passing
    private: ignition::transport::Node node;

    /// The model's entity
    private: ignition::gazebo::Entity modelEntity;

    /// The model's base link
    private: ignition::gazebo::Entity baseLink;

    /// The thruster link
    private: ignition::gazebo::Entity thrusterLink;

    /// The thruster joint
    private: ignition::gazebo::Entity thrusterJoint;

    /// The rudder joint
    private: ignition::gazebo::Entity rudderJoint;

    /// The elevator joint
    private: ignition::gazebo::Entity elevatorJoint;

    /// The mass shifter joint
    private: ignition::gazebo::Entity massShifterJoint;

    /// Publisher of robot state
    private: ignition::transport::Node::Publisher statePub;

    /// Publisher of thruster
    private: ignition::transport::Node::Publisher thrusterPub;

    /// Publisher of rudder position
    private: ignition::transport::Node::Publisher rudderPub;

    /// Publisher of elevator position
    private: ignition::transport::Node::Publisher elevatorPub;

    /// Publisher of mass shifter position
    private: ignition::transport::Node::Publisher massShifterPub;

    /// Publisher of buoyancy engine
    private: ignition::transport::Node::Publisher buoyancyEnginePub;

    /// Publisher of drop weight release
    private: ignition::transport::Node::Publisher dropWeightPub;
  };
}

#endif
