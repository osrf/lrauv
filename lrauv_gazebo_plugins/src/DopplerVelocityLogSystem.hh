/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef TETHYS_DOPPLERVELOCITYLOGSYSTEM_H_
#define TETHYS_DOPPLERVELOCITYLOGSYSTEM_H_

#include <gz/sim/System.hh>

namespace tethys
{

class DopplerVelocityLogSystemPrivate;

/// \brief System that creates and updates DopplerVelocityLog (DVL) sensors.
class DopplerVelocityLogSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: DopplerVelocityLogSystem();

  public: ~DopplerVelocityLogSystem();

  /// Inherits documentation from parent class
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr
  ) override;

  /// Inherits documentation from parent class
  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  /// Inherits documentation from parent class
  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<DopplerVelocityLogSystemPrivate> dataPtr;
};

}  // namespace tethys

#endif // TETHYS_DOPPLERVELOCITYLOGSYSTEM_H_
