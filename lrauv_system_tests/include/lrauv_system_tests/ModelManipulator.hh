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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#ifndef LRAUV_SYSTEM_TESTS__MODEL_MANIPULATOR_HH
#define LRAUV_SYSTEM_TESTS__MODEL_MANIPULATOR_HH

#include <optional>
#include <string>

#include <gz/sim/EntityComponentManager.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

namespace lrauv_system_tests {

/// Helper class to manipulate a model in an Ignition Gazebo simulation.
class ModelManipulator
{
  /// Constructor.
  /// \param[in] _modelName Name of the model to manipulate.
  public: ModelManipulator(const std::string &_modelName);

  /// Update simulation state as necessary.
  /// \note To be called on world pre-update.
  /// \param[in] _ecm Entity component manager to update.
  public: void Update(gz::sim::EntityComponentManager &_ecm);

  /// Set model orientation in simulation.
  /// \param[in] _orientation Model orientation in the world frame.
  public: void SetOrientation(const gz::math::Quaterniond &_orientation);

  private: std::string modelName;

  private: std::optional<gz::math::Pose3d> poseRequest;
};

}

#endif // LRAUV_SYSTEM_TESTS__MODEL_MANIPULATOR_HH
