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

#ifndef WORLD_COMM_PLUGIN_H_
#define WORLD_COMM_PLUGIN_H_

#include <chrono>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/math/Temperature.hh>
#include <ignition/transport/Node.hh>

#include "lrauv_command.pb.h"
#include "lrauv_init.pb.h"

namespace tethys
{
  /// Listens to LRAUVInit messages from the controller to spawn vehicles
  /// into simulation.
  class WorldCommPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure
  {
    // Documentation inherited
    public: void Configure(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr) override;

    /// Callback function for initialization message from LRAUV Main Vehicle
    /// Application
    /// \param[in] _msg Spawn message
    public: void SpawnCallback(
                const lrauv_ignition_plugins::msgs::LRAUVInit &_msg);

    /// Get the SDF string for a Tethys model with the given ID.
    /// \param[in] _id Used as model name and namespace for topics.
    /// \return SDF string
    public: std::string TethysSdfString(const std::string &_id);

    /// Generic callback to handle service responses.
    /// \param[in] _rep Response
    /// \param[in] _result True if the service was handled correctly
    private: void ServiceResponse(const ignition::msgs::Boolean &_rep,
        const bool _result);

    /// Service callback for world origin in spherical coordinates. Service is
    /// only valid after a first vehicle has been spawned.
    /// \param[out] _res World origin in spherical coordinates
    private: bool WorldOriginSphericalService(ignition::msgs::Vector3d &_res);

    /// Topic used to spawn robots
    private: std::string spawnTopic{"lrauv/init"};

    /// Transport node for message passing
    private: ignition::transport::Node node;

    /// Service to set spherical coordinates
    private: std::string setSphericalCoordsService;

    /// Service to create entities
    private: std::string createService;

    /// Service to get world origin spherical coordinates, after a first
    /// vehicle has been spawned.
    private: std::string getWorldOriginSphericalService;

    /// Count how many vehicles have been spawned
    private: int spawnCount{0};

    /// World origin in spherical coordinates (latitude, longitude, elevation)
    private: ignition::math::Vector3d worldOriginSphericalCoords;
  };
}

#endif
