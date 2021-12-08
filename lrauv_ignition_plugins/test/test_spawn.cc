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
  ignition::common::Console::SetVerbosity(4);

  // Setup fixture
  auto fixture = std::make_unique<ignition::gazebo::TestFixture>(
      ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "worlds", "empty_environment.sdf"));

  ignition::gazebo::Entity vehicle1{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity vehicle2{ignition::gazebo::kNullEntity};
  unsigned int iterations{0u};
  std::vector<ignition::math::Pose3d> poses1;
  std::vector<ignition::math::Pose3d> poses2;
  std::vector<ignition::math::Vector3d> latLon1;
  std::vector<ignition::math::Vector3d> latLon2;

  fixture->OnPostUpdate(
    [&](const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto worldEntity = ignition::gazebo::worldEntity(_ecm);
      ignition::gazebo::World world(worldEntity);

      vehicle1 = world.ModelByName(_ecm, "vehicle1");
      if (ignition::gazebo::kNullEntity != vehicle1)
      {
        poses1.push_back(ignition::gazebo::worldPose(vehicle1, _ecm));

        auto latLon = ignition::gazebo::sphericalCoordinates(vehicle1, _ecm);
        EXPECT_TRUE(latLon);
        latLon1.push_back(latLon.value());
      }

      vehicle2 = world.ModelByName(_ecm, "vehicle2");
      if (ignition::gazebo::kNullEntity != vehicle2)
      {
        poses2.push_back(ignition::gazebo::worldPose(vehicle2, _ecm));

        auto latLon = ignition::gazebo::sphericalCoordinates(vehicle2, _ecm);
        EXPECT_TRUE(latLon);
        latLon2.push_back(latLon.value());
      }

      iterations++;
    });
  fixture->Finalize();

  // Check that vehicles don't exist
  fixture->Server()->Run(true, 100, false);
  EXPECT_EQ(ignition::gazebo::kNullEntity, vehicle1);
  EXPECT_EQ(ignition::gazebo::kNullEntity, vehicle2);
  EXPECT_EQ(100, iterations);
  EXPECT_EQ(0, poses1.size());
  EXPECT_EQ(0, poses2.size());
  EXPECT_EQ(0, latLon1.size());
  EXPECT_EQ(0, latLon2.size());

  // Spawn first vehicle
  ignition::transport::Node node;
  auto spawnPub = node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInit>(
    "/lrauv/init");

  int sleep{0};
  int maxSleep{30};
  for (; !spawnPub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_LT(sleep, maxSleep);

  ignition::math::Angle lat1 = IGN_DTOR(20.0);
  ignition::math::Angle lon1 = IGN_DTOR(20.0);
  {
    lrauv_ignition_plugins::msgs::LRAUVInit spawnMsg;
    spawnMsg.mutable_id_()->set_data("vehicle1");
    spawnMsg.set_initlat_(lat1.Degree());
    spawnMsg.set_initlon_(lon1.Degree());

    spawnPub.Publish(spawnMsg);
  }

  // Check that vehicle was spawned
  fixture->Server()->Run(true, 100, false);
  EXPECT_NE(ignition::gazebo::kNullEntity, vehicle1);
  EXPECT_EQ(ignition::gazebo::kNullEntity, vehicle2);
  EXPECT_EQ(200, iterations);
  EXPECT_LT(50, poses1.size());
  EXPECT_EQ(0, poses2.size());
  EXPECT_LT(50, latLon1.size());
  EXPECT_EQ(0, latLon2.size());

  // Spawn second vehicle

  ignition::math::Angle lat2 = IGN_DTOR(20.1);
  ignition::math::Angle lon2 = IGN_DTOR(20.1);
  {
    lrauv_ignition_plugins::msgs::LRAUVInit spawnMsg;
    spawnMsg.mutable_id_()->set_data("vehicle2");
    spawnMsg.set_initlat_(lat2.Degree());
    spawnMsg.set_initlon_(lon2.Degree());

    spawnPub.Publish(spawnMsg);
  }

  // FIXME(anyone): Ideally all axes would have a tight tolerance, but depth
  // and pitch are currently unstable, see
  // https://github.com/osrf/lrauv/issues/49
  double tightTol{1e-5};
  double depthTol{4e-2};
  double pitchTol{1e-2};

  // Check that vehicles were spawned
  fixture->Server()->Run(true, 100, false);
  EXPECT_NE(ignition::gazebo::kNullEntity, vehicle1);
  EXPECT_NE(ignition::gazebo::kNullEntity, vehicle2);
  EXPECT_EQ(300, iterations);
  EXPECT_LT(150, poses1.size());
  EXPECT_LT(50, poses2.size());
  EXPECT_LT(150, latLon1.size());
  EXPECT_LT(50, latLon2.size());

  // Check vehicle positions
  EXPECT_NEAR(0.0, poses1.back().Pos().X(), tightTol);
  EXPECT_NEAR(0.0, poses1.back().Pos().Y(), tightTol);
  EXPECT_NEAR(0.0, poses1.back().Pos().Z(), depthTol);
  EXPECT_NEAR(0.0, poses1.back().Rot().Roll(), tightTol);
  EXPECT_NEAR(0.0, poses1.back().Rot().Pitch(), pitchTol);
  EXPECT_NEAR(0.0, poses1.back().Rot().Yaw(), tightTol);

  EXPECT_NEAR(lat1.Degree(), latLon1.back().X(), tightTol);
  EXPECT_NEAR(lon1.Degree(), latLon1.back().Y(), tightTol);
  EXPECT_NEAR(0.0, latLon1.back().Z(), depthTol);

  ignition::math::SphericalCoordinates sc;
  sc.SetLatitudeReference(lat1);
  sc.SetLongitudeReference(lon1);
  auto expectedPos2 = sc.PositionTransform(
      {lat2.Radian(), lon2.Radian(), 0.0},
      ignition::math::SphericalCoordinates::SPHERICAL,
      ignition::math::SphericalCoordinates::LOCAL2);
  EXPECT_NEAR(expectedPos2.X(), poses2.back().Pos().X(), tightTol);
  EXPECT_NEAR(expectedPos2.Y(), poses2.back().Pos().Y(), tightTol);
  EXPECT_NEAR(expectedPos2.Z(), poses2.back().Pos().Z(), depthTol);
  EXPECT_NEAR(0.0, poses2.back().Rot().Roll(), tightTol);
  EXPECT_NEAR(0.0, poses2.back().Rot().Pitch(), pitchTol);
  EXPECT_NEAR(0.0, poses2.back().Rot().Yaw(), tightTol);

  EXPECT_NEAR(lat2.Degree(), latLon2.back().X(), tightTol);
  EXPECT_NEAR(lon2.Degree(), latLon2.back().Y(), tightTol);
  EXPECT_NEAR(0.0, latLon2.back().Z(), depthTol);
}

