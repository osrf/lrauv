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

#ifndef LRAUV_SYSTEM_TESTS__MODEL_OBSERVER_HH
#define LRAUV_SYSTEM_TESTS__MODEL_OBSERVER_HH

#include <chrono>
#include <string>
#include <deque>

#include <ignition/gazebo/EntityComponentManager.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace lrauv_system_tests
{

/// Helper class to observe a model in an Ignition Gazebo simulation.
class ModelObserver
{
  /// Constructor.
  /// \param[in] _modelName Name of the model to observe.
  /// \param[in] _baseLinkName Name of the model's base link
  /// (to be used to measure linear and angular velocity).
  public: ModelObserver(
    const std::string &_modelName,
    const std::string &_baseLinkName = "base_link");

  /// Limit buffers' window size.
  /// \param[in] _windowSize Maximum window size as a duration
  /// measured against the simulation clock.
  public: void LimitTo(std::chrono::steady_clock::duration _windowSize);

  /// Update internal state from simulation state.
  /// \note To be called on world post-update.
  /// \param[in] _info Info for the current iteration.
  /// \param[in] _ecm Entity component manager to be queried.
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
                      const ignition::gazebo::EntityComponentManager &_ecm);

  /// Returns simulation times at which the model was observed.
  public: const std::deque<std::chrono::steady_clock::duration> &Times() const;

  /// Returns model world poses seen.
  public: const std::deque<ignition::math::Pose3d> &Poses() const;

  /// Returns model spherical coordinates seen.
  public:
  const std::deque<ignition::math::Vector3d> &SphericalCoordinates() const;

  /// Returns model linear velocities seen.
  public:
  const std::deque<ignition::math::Vector3d> &LinearVelocities() const;

  /// Returns model angular velocities seen.
  public:
  const std::deque<ignition::math::Vector3d> &AngularVelocities() const;

  private: std::string modelName;

  private: std::string baseLinkName;

  private: std::chrono::steady_clock::duration windowSize{
    std::chrono::steady_clock::duration::zero()};

  private: std::deque<std::chrono::steady_clock::duration> times;

  private: std::deque<ignition::math::Pose3d> poses;

  private: std::deque<ignition::math::Vector3d> sphericalCoordinates;

  private: std::deque<ignition::math::Vector3d> linearVelocities;

  private: std::deque<ignition::math::Vector3d> angularVelocities;
};

}

#endif // LRAUV_SYSTEM_TESTS__MODEL_OBSERVER_HH
