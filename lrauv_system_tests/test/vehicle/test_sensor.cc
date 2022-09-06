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

#include <gz/msgs/float.pb.h>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/transport/Node.hh>

#include <lrauv_ignition_plugins/lrauv_init.pb.h>

#include "TestConstants.hh"

using namespace std::chrono_literals;

std::atomic<bool> received_msg[4] = {false};

void ChlorophyllVeh1Cb(const gz::msgs::Float &_msg)
{
  // 0.22934943324714 is the value at this point. Allow some noise.
  EXPECT_NEAR(_msg.data(), 0.229, 0.001);
  received_msg[0] = true;
}

void ChlorophyllVeh2Cb(const gz::msgs::Float &_msg)
{
  // 0.3935107401546 is the value at this point. Allow some noise.
  EXPECT_NEAR(_msg.data(), 0.393, 0.001);
  received_msg[1] = true;
}

void ChlorophyllVeh3Cb(const gz::msgs::Float &_msg)
{
  // Nearest readings are:
  // Chlorophyll sensor: 0.935823@36.8 -122.72 40
  // Chlorophyll sensor: 0.229349@36.8 -122.72 50
  // Chlorophyll sensor: 0.576666@36.8 -122.7 40
  // Chlorophyll sensor: 0.946913@36.8 -122.7 50
  // Therefore value should lie between 0.94 and 0.229
  EXPECT_GT(_msg.data(), 0.229);
  EXPECT_LT(_msg.data(), 0.95);
  received_msg[2] = true;
}

void ChlorophyllVeh4Cb(const gz::msgs::Float &_msg)
{
  // Nearest readings are:
  // Chlorophyll sensor: 0.935823@36.8 -122.72 40
  //Chlorophyll sensor: 0.229349@36.8 -122.72 50
  //Chlorophyll sensor: 0.576666@36.8 -122.7 40
  //Chlorophyll sensor: 0.946913@36.8 -122.7 50
  // Therefore value should lie between 0.95 and 0.229
  EXPECT_GT(_msg.data(), 0.229);
  EXPECT_LT(_msg.data(), 0.95);
  received_msg[3] = true;
}

void SpawnVehicle(
  gz::transport::Node::Publisher &_spawnPub,
  const std::string &_modelName,
  const double _lat, const double _lon, const double _depth,
  const int _acommsAddr)
{
  gz::math::Angle lat1 = GZ_DTOR(_lat);
  gz::math::Angle lon1 = GZ_DTOR(_lon);

  lrauv_ignition_plugins::msgs::LRAUVInit spawnMsg;
  spawnMsg.mutable_id_()->set_data(_modelName);
  spawnMsg.set_initlat_(lat1.Degree());
  spawnMsg.set_initlon_(lon1.Degree());
  spawnMsg.set_initz_(_depth);
  spawnMsg.set_acommsaddress_(_acommsAddr);

  _spawnPub.Publish(spawnMsg);

}

//////////////////////////////////////////////////
TEST(SensorTest, PositionInterpolation)
{
  gz::common::Console::SetVerbosity(4);

  // Setup fixture
  auto fixture = std::make_unique<gz::sim::TestFixture>(
      gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "worlds", "empty_environment.sdf"));
  unsigned int iterations{0u};

  bool spawnedAllVehicles = {false};
  static const std::string vehicles[] =
    {"vehicle1", "vehicle2", "vehicle3", "vehicle4"};

  fixture->OnPostUpdate(
    [&](const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
    {
      auto worldEntity = gz::sim::worldEntity(_ecm);
      gz::sim::World world(worldEntity);
      int numVehiclesSpawned{0};
      for (auto v1 : vehicles)
      {
        if (world.ModelByName(_ecm, v1) != gz::sim::kNullEntity)
        {
          numVehiclesSpawned++;
        }
      }

      if (numVehiclesSpawned == 4)
      {
        spawnedAllVehicles = true;
      }
      iterations++;
    });
  fixture->Finalize();

  // Check that vehicles don't exist
  fixture->Server()->RunOnce();
  EXPECT_EQ(1, iterations);
  // Spawn first vehicle
  gz::transport::Node node;
  auto spawnPub = node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInit>(
    "/lrauv/init");

  int sleep{0};
  int maxSleep{30};
  for (; !spawnPub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_LE(sleep, maxSleep);


  // On a depth coplanar to a piece of data
  SpawnVehicle(
    spawnPub, "vehicle1", 36.7999992370605, -122.720001220703, 50, 0);
  node.Subscribe("/model/vehicle1/chlorophyll", &ChlorophyllVeh1Cb);

  // At the surface should still have reading
  SpawnVehicle(
    spawnPub, "vehicle2", 36.7999992370605, -122.720001220703, 0, 0);
  node.Subscribe("/model/vehicle2/chlorophyll", &ChlorophyllVeh2Cb);

  // In the middle of two slices, still in a plane
  SpawnVehicle(
    spawnPub, "vehicle3", 36.7999992370605, -122.720001220703, 45, 0);
  node.Subscribe("/model/vehicle3/chlorophyll", &ChlorophyllVeh3Cb);

  // In the middle of a quadrant
  SpawnVehicle(
    spawnPub, "vehicle4", 36.7999992370605, -122.7, 48, 0);
  node.Subscribe("/model/vehicle4/chlorophyll", &ChlorophyllVeh4Cb);

  // Check that vehicle was spawned
  int expectedIterations = iterations;
  for (sleep = 0; !spawnedAllVehicles && sleep < maxSleep;
      ++sleep)
  {
    std::this_thread::sleep_for(100ms);
    // Run paused so we avoid the physics moving the vehicles
    fixture->Server()->RunOnce(true);
    expectedIterations++;
    igndbg << "Waiting for vehicles to spawn" << std::endl;
  }
  EXPECT_TRUE(spawnedAllVehicles);

  fixture->Server()->Run(true, 10, false);

  for (auto &msg: received_msg)
  {
    EXPECT_TRUE(msg.load());
  }
}
