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
#ifndef TETHYS_COMM_PLUGIN_H_
#define TETHYS_COMM_PLUGIN_H_

#include <chrono>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/transport/Node.hh>

#include "lrauv_command.pb.h"

namespace tethys_comm_plugin
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

    /// Callback function for command
    /// \param[in] _msg Command message
    public: void CommandCallback(
                const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg);

    /// Callback function for buoyancy bladder state
    /// \param[in] _msg Bladder volume
    public: void BuoyancyStateCallback(
                const ignition::msgs::Double &_msg);

    private: void SetupEntities(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr);

    /// Set up control message topics
    /// \param[in] _ns Namespace to prepend to topic names
    private: void SetupControlTopics(const std::string &_ns);

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

    /// Topic to publish for dropweight
    private: std::string dropWeightTopic
      {"drop_weight"};

    /// Model name
    private: std::string baseLinkName{"base_link"};

    /// Propeller link name
    private: std::string thrusterLinkName{"propeller"};

    /// Rudder joint name
    private: std::string rudderJointName{"vertical_fins_joint"};

    /// Elevator joint name
    private: std::string elevatorJointName{"horizontal_fins_joint"};

    /// Mass shifter joint name
    private: std::string massShifterJointName{"battery_joint"};

    /// TODO(mabelzhang) Remove when stable. Temporary counter for state
    ///  message sanity check
    private: int counter = 0;

    /// Buoyancy bladder size in cc
    private: double buoyancyBladderVolume = 300;

    /// TODO(mabelzhang) Remove when stable. Temporary timers for state message
    /// sanity check
    private: std::chrono::steady_clock::duration prevPubPrintTime =
      std::chrono::steady_clock::duration::zero();
    private: std::chrono::steady_clock::duration prevSubPrintTime =
      std::chrono::steady_clock::duration::zero();

    /// Transport node for message passing
    private: ignition::transport::Node node;

    /// Spherical coordinate handler
    private: ignition::math::SphericalCoordinates sphericalCoords;

    /// The model in question
    private: ignition::gazebo::Entity modelLink;

    /// The thruster link
    private: ignition::gazebo::Entity thrusterLink;

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

#endif /*TETHYS_COMM_PLUGIN_H_*/
