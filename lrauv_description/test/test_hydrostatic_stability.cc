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

#include "TestConstants.hh"
#include <gtest/gtest.h>

#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

TEST(Stability, FlatWorld)
{
  auto fixture = std::make_unique<ignition::gazebo::TestFixture>(
    ignition::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds", "flat_world.sdf"));

  std::vector<ignition::math::Pose3d> tethysPoses;

  fixture->OnPostUpdate(
    [&](const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto worldEntity = ignition::gazebo::worldEntity(_ecm);
      ignition::gazebo::World world(worldEntity);

      auto modelEntity = world.ModelByName(_ecm, "tethys");
      EXPECT_NE(ignition::gazebo::kNullEntity, modelEntity);

      tethysPoses.push_back(
          ignition::gazebo::worldPose(modelEntity, _ecm));
    }
  );

  fixture->Finalize();

  fixture->Server()->Run(true, 10000, false);
  EXPECT_EQ(10000, tethysPoses.size());

  bool first = true;
  ignition::math::Pose3d prev_pose;
  for (const auto &pose: tethysPoses)
  {
    if (first)
    {
      prev_pose = pose;
      first = false;
      continue;
    }
    EXPECT_EQ(prev_pose, pose);
    prev_pose = pose;
  }
  EXPECT_EQ(prev_pose.Pos(), ignition::math::Vector3d(0,0,0));
  EXPECT_EQ(prev_pose.Rot(), ignition::math::Quaterniond(1,0,0,0));
}

TEST(Stability, TiltedWorld)
{
  auto fixture = std::make_unique<ignition::gazebo::TestFixture>(
    ignition::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds", "tilted_world.sdf"));

  std::vector<ignition::math::Pose3d> tethysPoses;

  fixture->OnPostUpdate(
    [&](const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto worldEntity = ignition::gazebo::worldEntity(_ecm);
      ignition::gazebo::World world(worldEntity);

      auto modelEntity = world.ModelByName(_ecm, "tethys");
      EXPECT_NE(ignition::gazebo::kNullEntity, modelEntity);

      tethysPoses.push_back(
          ignition::gazebo::worldPose(modelEntity, _ecm));
    }
  );

  fixture->Finalize();

  fixture->Server()->Run(true, 100000, false);
  EXPECT_EQ(100000, tethysPoses.size());

  bool first = true;

  double maxPitch {std::numeric_limits<double>::min()},
    minPitch{std::numeric_limits<double>::max()};
  ignition::math::Pose3d prev_pose;
  for (const auto &pose: tethysPoses)
  {
    if (first)
    {
      prev_pose = pose;
      first = false;
      continue;
    }
    EXPECT_NEAR(prev_pose.Pos().X(), pose.Pos().X(), 1e-2);
    EXPECT_NEAR(prev_pose.Pos().Y(), pose.Pos().Y(), 1e-2);
    EXPECT_NEAR(prev_pose.Pos().Z(), pose.Pos().Z(), 1e-2);
    auto pitch = pose.Rot().Euler().Y();
    if (pitch > maxPitch)
    {
      maxPitch = pitch;
    }

    if (pitch < minPitch)
    {
      minPitch = pitch;
    }

    prev_pose = pose;
  }

  // Since we start the system at 0.08 pitch, we should not exceed this.
  EXPECT_LE(maxPitch, 0.08);
}
