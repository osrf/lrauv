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

/*
* This test evaluates whether the 
*
*/

#include <chrono>
#include <thread>
#include <gtest/gtest.h>

#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/transport/Node.hh>

#include "lrauv_init.pb.h"

#include "TestConstants.hh"

//////////////////////////////////////////////////
TEST(SpawnTest, Spawn)
{
  using namespace ignition;
  common::Console::SetVerbosity(4);

  // Setup fixture
  auto fixture = std::make_unique<ignition::gazebo::TestFixture>(
      ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "worlds", "empty_environment.sdf"));

  gazebo::Entity vehicle1{gazebo::kNullEntity};
  gazebo::Entity vehicle2{gazebo::kNullEntity};
  gazebo::Entity vehicle3{gazebo::kNullEntity};
  gazebo::Entity vehicle4{gazebo::kNullEntity};

  std::vector<math::Vector3d> velocitiesV1;
  std::vector<math::Vector3d> velocitiesV2;
  std::vector<math::Vector3d> velocitiesV3;
  std::vector<math::Vector3d> velocitiesV4;
  
  std::vector<math::Pose3d> posesV1;
  std::vector<math::Pose3d> posesV2;
  std::vector<math::Pose3d> posesV3;
  std::vector<math::Pose3d> posesV4;

  fixture->OnPostUpdate(
    [&](const gazebo::UpdateInfo &_info,
    const gazebo::EntityComponentManager &_ecm)
    {
      auto worldEntity = gazebo::worldEntity(_ecm);
      gazebo::World world(worldEntity);

      vehicle1 = world.ModelByName(_ecm, "tethys");
      if (gazebo::kNullEntity != vehicle1)
      {
        auto baselink1 = Model(vehicle1).LinkByName(_ecm, "base_link");
        gazebo::Link link(baselink1);
        auto velocity = link.WorldLinearVelocity(_ecm);
        auto pose = link.WorldPose(_ecm);
      }

      vehicle2 = world.ModelByName(_ecm, "thetys2");
      if (gazebo::kNullEntity != vehicle2)
      {
      }

      vehicle3 = world.ModelByName(_ecm, "tethys3");
      if (gazebo::kNullEntity != vehicle3)
      {
      }

      vehicle4 = world.ModelByName(_ecm, "tethys4");
      auto baselink4 = vehicle1.LinkByName(_ecm, "base_link");
      if (ignition::gazebo::kNullEntity != vehicle4)
      {
      }

    });
  fixture->Finalize();

  // Check that vehicles don't exist
  fixture->Server()->Run(true, 100, false);
}