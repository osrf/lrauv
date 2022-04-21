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

using namespace std::chrono_literals;

void ChlorophyllVeh1Cb(const ignition::msgs::Float &_msg)
{
  // 0.22934943324714 is the value at this point. Allow some noise.
  EXPECT_NEAR(_msg.data(), 0.229, 0.001);
}

void SpawnVehicle(
  ignition::transport::Node::Publisher &_spawnPub,
  const std::string &_modelName,
  const double _lat, const double _lon, const double _depth,
  const int _acommsAddr)
{
  ignition::math::Angle lat1 = IGN_DTOR(_lat);
  ignition::math::Angle lon1 = IGN_DTOR(_lon);

  lrauv_ignition_plugins::msgs::LRAUVInit spawnMsg;
  spawnMsg.mutable_id_()->set_data(_modelName);
  spawnMsg.set_initlat_(lat1.Degree());
  spawnMsg.set_initlon_(lon1.Degree());
  spawnMsg.set_initz_(_depth);
  spawnMsg.set_acommsaddress_(_acommsAddr);

  _spawnPub.Publish(spawnMsg);

}

//////////////////////////////////////////////////
TEST(SensorTest, Sensor)
{
  ignition::common::Console::SetVerbosity(4);

  // Setup fixture
  auto fixture = std::make_unique<ignition::gazebo::TestFixture>(
      ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "worlds", "empty_environment.sdf"));

  ignition::gazebo::Entity vehicle1{ignition::gazebo::kNullEntity};
  unsigned int iterations{0u};

  std::unordered_set<std::string> vehicles;

  fixture->OnPostUpdate(
    [&](const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto worldEntity = ignition::gazebo::worldEntity(_ecm);
      ignition::gazebo::World world(worldEntity);
      vehicle1 = world.ModelByName(_ecm, "vehicle1");
      iterations++;
    });
  fixture->Finalize();

  // Check that vehicles don't exist
  fixture->Server()->RunOnce();
  EXPECT_EQ(ignition::gazebo::kNullEntity, vehicle1);
  EXPECT_EQ(1, iterations);
  // Spawn first vehicle
  ignition::transport::Node node;
  node.Subscribe("/model/vehicle1/chlorophyll", &ChlorophyllVeh1Cb);
  auto spawnPub = node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInit>(
    "/lrauv/init");

  int sleep{0};
  int maxSleep{30};
  for (; !spawnPub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_LE(sleep, maxSleep);

  double spawnDepth{50};

  // On a depth coplanar to a peice of data
  SpawnVehicle(
    spawnPub, "vehicle1", 36.7999992370605, -122.720001220703, 50, 0);

  // At the surface should still have reading
  SpawnVehicle(
    spawnPub, "vehicle2", 36.7999992370605, -122.720001220703, 0, 0);

  // In the middle of two slices
  SpawnVehicle(
    spawnPub, "vehicle3", 36.7999992370605, -122.720001220703, 48, 0);

  // In the middle of a quadrant
  SpawnVehicle(
    spawnPub, "vehicle4", 36.7999992370605, -122.7, 48, 0);

  // Check that vehicle was spawned
  int expectedIterations = iterations;
  for (sleep = 0; ignition::gazebo::kNullEntity == vehicle1 && sleep < maxSleep;
      ++sleep)
  {
    std::this_thread::sleep_for(100ms);
    // Run paused so we avoid the physics moving the vehicles
    fixture->Server()->RunOnce(true);
    expectedIterations++;
    igndbg << "Waiting for vehicle1" << std::endl;
  }


  fixture->Server()->Run(true, 10000, false);

}
