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

#include <gz/msgs/double.pb.h>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/transport/Node.hh>

#include <lrauv_gazebo_plugins/lrauv_init.pb.h>

#include "TestConstants.hh"

using namespace std::chrono_literals;

std::atomic<std::chrono::steady_clock::duration> duration;

///////////////////////////////////////////////
void TemperatureVeh1Cb(const gz::msgs::Double &_msg)
{
  auto timeNow = std::chrono::duration<double>(duration.load()).count();
  auto temperature = _msg.data();

  if (timeNow < 10)
  {
    // At time 0 according to the csv file the temperature is 5.
    // Using linear interpolation the temparature at timeNow will be
    // (10 * timeNow + (10-timeNow) * 5) / 10
    EXPECT_NEAR(temperature, (10 * timeNow + (10-timeNow) * 5) / 10, 0.1);
  }
  else
  {
    // At time 10 according to the csv file temperature is 10.
    // Since theres no more data we will just keep the temperature as is.
    EXPECT_NEAR(temperature, 10.0, 0.1);
  }
}

///////////////////////////////////////////////
void SpawnVehicle(
  gz::transport::Node::Publisher &_spawnPub,
  const std::string &_modelName,
  const double _lat, const double _lon, const double _depth,
  const int _acommsAddr)
{
  gz::math::Angle lat1 = GZ_DTOR(_lat);
  gz::math::Angle lon1 = GZ_DTOR(_lon);

  lrauv_gazebo_plugins::msgs::LRAUVInit spawnMsg;
  spawnMsg.mutable_id_()->set_data(_modelName);
  spawnMsg.set_initlat_(lat1.Degree());
  spawnMsg.set_initlon_(lon1.Degree());
  spawnMsg.set_initz_(_depth);
  spawnMsg.set_acommsaddress_(_acommsAddr);

  _spawnPub.Publish(spawnMsg);
}

//////////////////////////////////////////////////
TEST(SensorTest, TimeInterpolation)
{
  gz::common::Console::SetVerbosity(4);

  // Setup fixture
  auto fixture = std::make_unique<gz::sim::TestFixture>(
      gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "worlds", "empty_environment.sdf"));
  unsigned int iterations{0u};

  bool spawnedAllVehicles = {false};
  static const std::string vehicles[] =
    {"vehicle1"};

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

      if (numVehiclesSpawned == 1)
      {
        spawnedAllVehicles = true;
      }
      iterations++;
      duration = _info.simTime;
    });
  fixture->Finalize();

  // Check that vehicles don't exist
  fixture->Server()->RunOnce();
  EXPECT_EQ(1, iterations);

  int sleep{0};
  int maxSleep{30};

  // Change the dataconfig
  gzdbg << "Switching file" <<std::endl;
  gz::transport::Node node;
  auto configPub = node.Advertise<gz::msgs::StringMsg>(
    "/world/science_sensor/environment_data_path");
  gz::msgs::StringMsg configMsg;
  configMsg.set_data(gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "data", "minimal_time_varying.csv"));
  for (; !configPub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(100ms);
  }
  configPub.Publish(configMsg);

  // Spawn first vehicle
  auto spawnPub = node.Advertise<lrauv_gazebo_plugins::msgs::LRAUVInit>(
    "/lrauv/init");
  for (; !spawnPub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_LE(sleep, maxSleep);


  // On a depth coplanar to a piece of data
  SpawnVehicle(
    spawnPub, "vehicle1", 0.000005, 0.000005, 5, 0);
  node.Subscribe("/model/vehicle1/temperature", &TemperatureVeh1Cb);

  // Check that vehicle was spawned
  int expectedIterations = iterations;
  for (sleep = 0; !spawnedAllVehicles && sleep < maxSleep;
      ++sleep)
  {
    std::this_thread::sleep_for(100ms);
    // Run paused so we avoid the physics moving the vehicles
    fixture->Server()->RunOnce(true);
    expectedIterations++;
    gzdbg << "Waiting for vehicles to spawn" << std::endl;
  }
  EXPECT_TRUE(spawnedAllVehicles);

  fixture->Server()->Run(true, 1000, false);
}
