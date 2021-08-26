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

#ifndef TETHYS_SCIENCESENSORSSYSTEM_
#define TETHYS_SCIENCESENSORSSYSTEM_

#include <unordered_map>

#include <ignition/common/Console.hh>
#include <ignition/math/Temperature.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/CustomSensor.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/Util.hh>

#include "LookupSensor.hh"

namespace tethys
{
/// \brief Sensor that detects and publishes salinity values in PSU.
LOOKUP_SENSOR(SalinitySensor, float, salinity);

/// \brief Sensor that detects temperature, publishes in Celsius.
LOOKUP_SENSOR(TemperatureSensor, ignition::math::Temperature, temperature);

/// \brief Sensor that detects and publishes chlorophyll values in ug / L.
LOOKUP_SENSOR(ChlorophyllSensor, float, chlorophyll);

/// \brief Sensor that detects 3D current velocities in m / s.
/// X: East
/// Y: North
/// Z: Up
LOOKUP_SENSOR(CurrentSensor, ignition::math::Vector3d, current);

/// \brief System that creates and updates the science sensors defined above.
class ScienceSensorsSystem:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate
{
  // Documentation inherited
  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

  // Documentation inherited.
  public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) final;

  /// \brief Remove sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  private: void RemoveSensorEntities(
      const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief A map of entities to their sensors
  public: std::unordered_map<ignition::gazebo::Entity,
      std::shared_ptr<ignition::sensors::Sensor>> entitySensorMap;
};
}
#endif
