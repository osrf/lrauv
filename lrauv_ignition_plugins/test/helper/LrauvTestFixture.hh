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
#ifndef LRAUVIGNITIONPLUGINS_LRAUVTESTFIXTURE_HH_
#define LRAUVIGNITIONPLUGINS_LRAUVTESTFIXTURE_HH_

#include <gtest/gtest.h>

#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include "lrauv_command.pb.h"

#include "TestConstants.hh"

using namespace std::chrono_literals;

/// \brief Convenient fixture that provides boilerplate code common to most
/// LRAUV tests.
class LrauvTestFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);

    // Setup transport
    auto commandTopic = "/tethys/command_topic";
    this->commandPub =
      this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(
      commandTopic);

    // Setup fixture
    this->fixture = std::make_unique<ignition::gazebo::TestFixture>(
        ignition::common::joinPaths(
        std::string(PROJECT_SOURCE_PATH), "worlds", "buoyant_tethys.sdf"));

    fixture->OnPostUpdate(
      [&](const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
      {
        auto worldEntity = ignition::gazebo::worldEntity(_ecm);
        ignition::gazebo::World world(worldEntity);

        auto modelEntity = world.ModelByName(_ecm, "tethys");
        EXPECT_NE(ignition::gazebo::kNullEntity, modelEntity);

        this->tethysPoses.push_back(
            ignition::gazebo::worldPose(modelEntity, _ecm));
        this->iterations++;
      });
    fixture->Finalize();
  }

  /// \brief Keep publishing a command while a condition is met or it times out
  /// \param[in] _msg Command to be published
  /// \param[in] _continue Function that returns true if we should keep
  /// publishing, that is, something that is initially true but should become
  /// false as the command is processed.
  public: void PublishCommandWhile(
      const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg,
      std::function<bool()> _continue)
  {
    int maxSleep{30};
    int sleep{0};
    for (; sleep < maxSleep && _continue(); ++sleep)
    {
      this->commandPub.Publish(_msg);
      this->fixture->Server()->Run(true, 300, false);
      std::this_thread::sleep_for(100ms);
    }
    EXPECT_LT(sleep, maxSleep);
  }

  /// \brief How many times has OnPostUpdate been run
  public: unsigned int iterations{0u};

  /// \brief All tethys world poses in order
  public: std::vector<ignition::math::Pose3d> tethysPoses;

  /// \brief Test fixture
  public: std::unique_ptr<ignition::gazebo::TestFixture> fixture{nullptr};

  /// \brief Node for communication
  public: ignition::transport::Node node;

  /// \brief Publishes commands
  public: ignition::transport::Node::Publisher commandPub;
};
#endif
