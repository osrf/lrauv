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

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

#include "lrauv_command.pb.h"

namespace tethys_comm_plugin
{
  class TethysCommPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// Constructor
    public: TethysCommPlugin();

    /// Destructor
    public: ~TethysCommPlugin() override = default;

    // Documentation inherited
    public: void Configure(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// Callback function for command
    /// \param[in] _msg Command message
    void CommandCallback(const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg);

    /// Topic on which robot commands will be received
    private: std::string commandTopic{"command_topic"};

    /// Topic on which robot state will be published
    private: std::string stateTopic{"state_topic"};

    /// TODO(mabelzhang) Remove when stable. Temporary counter for state message sanity check
    private: int counter = 0;

    /// TODO(mabelzhang) Remove when stable. Temporary timer for state message sanity check
    private: std::chrono::time_point<std::chrono::steady_clock> elapsed;

    /// Transport node for message passing
    private: ignition::transport::Node node;

    /// Publisher of robot state
    private: ignition::transport::Node::Publisher statePub;
  };
}

#endif /*TETHYS_COMM_PLUGIN_H_*/
