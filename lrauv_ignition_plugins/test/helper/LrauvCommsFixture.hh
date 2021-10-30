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

#ifndef LRAUVIGNITIONPLUGINS_LRAUVCOMMSFIXTURE_HH_
#define LRAUVIGNITIONPLUGINS_LRAUVCOMMSFIXTURE_HH_

#include <gtest/gtest.h>

#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include "TestConstants.hh"

using namespace std::chrono_literals;

/// \brief Convenient fixture that provides boilerplate code common to most
/// LRAUV tests.
class LrauvCommsFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);

    // Setup fixture
    this->fixture = std::make_unique<ignition::gazebo::TestFixture>(
        ignition::common::joinPaths(
        std::string(PROJECT_SOURCE_PATH), "worlds", "acoustic_comms_test.sdf"));

    fixture->OnPostUpdate(
      [&](const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
      {
        std::lock_guard<std::mutex> lock(this->mtx);
        this->iterations++;
        this->now = std::chrono::steady_clock::time_point(_info.simTime);
      });
    fixture->Finalize();
  }

   /// \brief Test fixture
  public: std::unique_ptr<ignition::gazebo::TestFixture> fixture{nullptr};

  /// \brief How many times has OnPostUpdate been run
  public: unsigned int iterations{0u};

  /// \brief Mutex variable
  public: std::mutex mtx;

  /// \brief Current time
  public: std::chrono::steady_clock::time_point now;
};
#endif
