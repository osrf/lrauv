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

#ifndef TETHYS_SCIENCESENSORSSYSTEM_
#define TETHYS_SCIENCESENSORSSYSTEM_

#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/math/Temperature.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>

#include "LookupSensor.hh"

namespace tethys
{
/// \brief Sensor that detects and publishes salinity values in PSU.
LOOKUP_SENSOR(SalinitySensor, float, salinity);

/// \brief Sensor that detects temperature, publishes in Celsius.
LOOKUP_SENSOR(TemperatureSensor, gz::math::Temperature, temperature);

/// \brief Sensor that detects and publishes chlorophyll values in ug / L.
LOOKUP_SENSOR(ChlorophyllSensor, float, chlorophyll);

/// \brief Sensor that detects 3D current velocities in m / s.
/// X: East
/// Y: North
/// Z: Up
LOOKUP_SENSOR(CurrentSensor, gz::math::Vector3d, current);

class ScienceSensorsSystemPrivate;

/// \brief System that creates and updates the science sensors defined above.
class ScienceSensorsSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: ScienceSensorsSystem();

  // Documentation inherited
  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) final;

  /// \brief Remove sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  private: void RemoveSensorEntities(
      const gz::sim::EntityComponentManager &_ecm);

  /// \brief A map of entities to their sensors
  public: std::unordered_map<gz::sim::Entity,
      std::shared_ptr<gz::sensors::Sensor>> entitySensorMap;

  /// \brief Private data pointer
  private: std::unique_ptr<ScienceSensorsSystemPrivate> dataPtr;
};
}
#endif
