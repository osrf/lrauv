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

#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/math/Temperature.hh>
#include <gz/transport/Node.hh>

#include "lrauv_gazebo_plugins/lrauv_command.pb.h"

namespace tethys
{
  class TethysCommPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    // Documentation inherited
    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// Callback function for command from LRAUV Main Vehicle Application
    /// \param[in] _msg Command message
    public: void CommandCallback(
                const lrauv_gazebo_plugins::msgs::LRAUVCommand &_msg);

    /// Callback function for buoyancy bladder state
    /// \param[in] _msg Bladder volume
    public: void BuoyancyStateCallback(
                const gz::msgs::Double &_msg);

    /// Callback function for salinity sensor data.
    /// \param[in] _msg Sensor data
    public: void SalinityCallback(
                const gz::msgs::Double &_msg);

    /// Callback function for temperature sensor data.
    /// \param[in] _msg Sensor data
    public: void TemperatureCallback(
                const gz::msgs::Double &_msg);

    /// Callback function for battery data.
    /// \param[in] _msg Battery data
    public: void BatteryCallback(
                const gz::msgs::BatteryState &_msg);

    /// Callback function for chlorophyll sensor data.
    /// \param[in] _msg Sensor data
    public: void ChlorophyllCallback(
                const gz::msgs::Double &_msg);

    /// Callback function for current sensor data.
    /// \param[in] _msg Sensor data
    public: void CurrentCallback(
                const gz::msgs::Vector3d &_msg);

    /// Parse SDF parameters and create components
    private: void SetupEntities(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr);

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
      {"propeller_joint/cmd_vel"};

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

    /// Topic to subscribe to battery data
    private: std::string batteryTopic
      {"battery/linear_battery/state"};

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
    private: gz::math::Temperature latestTemperature{std::nanf("")};

    /// Latest battery voltage data received, Nan if not received.
    private: double latestBatteryVoltage{std::nanf("")};

    /// Latest battery current data received, Nan if not received.
    private: double latestBatteryCurrent{std::nanf("")};

    /// Latest battery charge data received, Nan if not received.
    private: double latestBatteryCharge{std::nanf("")};

    /// Latest battery percentage data received, Nan if not received.
    private: double latestBatteryPercentage{std::nanf("")};

    /// Latest chlorophyll data received from sensor. NaN if not received.
    private: float latestChlorophyll{std::nanf("")};

    /// Ocean Density in kg / m ^ 3
    private: double oceanDensity{1000};

    /// Latest current data received from sensor. NaN if not received.
    private: gz::math::Vector3d latestCurrent
        {std::nan(""), std::nan(""), std::nan("")};

    /// TODO(mabelzhang) Remove when stable. Temporary timers for state message
    /// sanity check
    private: std::chrono::steady_clock::duration prevPubPrintTime =
      std::chrono::steady_clock::duration::zero();

    /// Transport node for message passing
    private: gz::transport::Node node;

    /// The model's entity
    private: gz::sim::Entity modelEntity;

    /// The model's base link
    private: gz::sim::Entity baseLink;

    /// The thruster link
    private: gz::sim::Entity thrusterLink;

    /// The thruster joint
    private: gz::sim::Entity thrusterJoint;

    /// The rudder joint
    private: gz::sim::Entity rudderJoint;

    /// The elevator joint
    private: gz::sim::Entity elevatorJoint;

    /// The mass shifter joint
    private: gz::sim::Entity massShifterJoint;

    /// Publisher of robot state
    private: gz::transport::Node::Publisher statePub;

    /// Publisher of robot NavSat location
    private: gz::transport::Node::Publisher navSatPub;

    /// Publisher of thruster
    private: gz::transport::Node::Publisher thrusterPub;

    /// Publisher of rudder position
    private: gz::transport::Node::Publisher rudderPub;

    /// Publisher of elevator position
    private: gz::transport::Node::Publisher elevatorPub;

    /// Publisher of mass shifter position
    private: gz::transport::Node::Publisher massShifterPub;

    /// Publisher of buoyancy engine
    private: gz::transport::Node::Publisher buoyancyEnginePub;

    /// Publisher of drop weight release
    private: gz::transport::Node::Publisher dropWeightPub;
  };
}

#endif
