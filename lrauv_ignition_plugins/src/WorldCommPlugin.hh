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

#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/math/Temperature.hh>
#include <gz/transport/Node.hh>

#include "lrauv_ignition_plugins/lrauv_init.pb.h"

namespace tethys
{
  /// Listens to LRAUVInit messages from the controller to spawn vehicles
  /// into simulation.
  ///
  /// If the world origin's spherical coordinates aren't set from SDF, this
  /// plugin will set them to match the first vehicle spawned.
  class WorldCommPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure
  {
    // Documentation inherited
    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    /// Callback function for initialization message from LRAUV Main Vehicle
    /// Application
    /// \param[in] _msg Spawn message
    public: void SpawnCallback(
                const lrauv_ignition_plugins::msgs::LRAUVInit &_msg);

    /// Get the SDF string for a Tethys model with given ID and acoustic modem address.
    /// \param[in] _msg LRAUV init message containing ID and acoustic modem address.
    /// \return SDF string
    public: std::string TethysSdfString(
                const lrauv_ignition_plugins::msgs::LRAUVInit &_msg);

    /// Generic callback to handle service responses.
    /// \param[in] _rep Response
    /// \param[in] _result True if the service was handled correctly
    private: void ServiceResponse(const gz::msgs::Boolean &_rep,
        const bool _result);

    /// Topic used to spawn robots
    private: std::string spawnTopic{"lrauv/init"};

    /// Transport node for message passing
    private: gz::transport::Node node;

    /// Service to set spherical coordinates
    private: std::string setSphericalCoordsService;

    /// Service to create entities
    private: std::string createService;

    /// Service to make spawned entities performers (for levels)
    private: std::string performerService;

    /// Whether the world origin's latitude and longitude have already been set.
    private: bool hasWorldLatLon{false};
  };
}

#endif
