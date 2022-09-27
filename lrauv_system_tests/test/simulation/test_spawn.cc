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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/math/Angle.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/transport/Node.hh>

#include <lrauv_gazebo_plugins/lrauv_init.pb.h>

#include "lrauv_system_tests/Publisher.hh"
#include "lrauv_system_tests/TestFixture.hh"
#include "lrauv_system_tests/Util.hh"

#include "TestConstants.hh"

using namespace lrauv_system_tests;
using namespace std::literals::chrono_literals;

class DynamicTestFixture : public TestFixture
{
  using TestFixture::TestFixture;

  public: ModelObserver &Observe(const std::string &_modelName)
  {
    this->vehicleObservers.push_back(
        std::make_unique<ModelObserver>(_modelName));
    return *this->vehicleObservers.back();
  }

  protected: void OnPostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override
  {
    for (auto &observer : this->vehicleObservers)
    {
      observer->Update(_info, _ecm);
    }
  }

  private: std::vector<std::unique_ptr<ModelObserver>> vehicleObservers;
};

//////////////////////////////////////////////////
TEST(VehicleSpawnTest, Spawn)
{
  DynamicTestFixture fixture(worldPath("empty_environment.sdf"));
  auto &observer1 = fixture.Observe("vehicle1");
  auto &observer2 = fixture.Observe("vehicle2");

  // Run paused so we avoid the physics moving the vehicles
  fixture.Pause();

  EXPECT_EQ(1, fixture.Step());
  EXPECT_EQ(0, observer1.Poses().size());
  EXPECT_EQ(0, observer1.SphericalCoordinates().size());
  EXPECT_EQ(0, observer2.Poses().size());
  EXPECT_EQ(0, observer2.SphericalCoordinates().size());

  // Spawn first vehicle
  gz::transport::Node node;
  using lrauv_gazebo_plugins::msgs::LRAUVInit;
  gz::transport::Node::Publisher spawnPublisher =
      node.Advertise<LRAUVInit>("/lrauv/init");
  ASSERT_TRUE(WaitForConnections(spawnPublisher, 5s));

  // No specific orientation, vehicle will face North
  const gz::math::Angle lat1 = GZ_DTOR(20.0);
  const gz::math::Angle lon1 = GZ_DTOR(20.0);
  {
    lrauv_gazebo_plugins::msgs::LRAUVInit spawnMessage;
    spawnMessage.mutable_id_()->set_data("vehicle1");
    spawnMessage.set_initlat_(lat1.Degree());
    spawnMessage.set_initlon_(lon1.Degree());
    spawnMessage.set_acommsaddress_(201);

    spawnPublisher.Publish(spawnMessage);

    Timeout timeout{3s};
    do {
      EXPECT_EQ(1, fixture.Step());
      std::this_thread::sleep_for(100ms);
    } while (observer1.Poses().empty() && !timeout);
  }

  EXPECT_LT(0, observer1.Poses().size());
  EXPECT_LT(0, observer1.SphericalCoordinates().size());
  EXPECT_EQ(0, observer2.Poses().size());
  EXPECT_EQ(0, observer2.SphericalCoordinates().size());

  // Spawn vehicle facing South
  // Orientation is in NED, so 180 degrees yaw is South
  const gz::math::Angle lat2 = GZ_DTOR(20.1);
  const gz::math::Angle lon2 = GZ_DTOR(20.1);
  const gz::math::Angle yaw2 = GZ_DTOR(180);
  const double depth2 = 10.0;
  {
    lrauv_gazebo_plugins::msgs::LRAUVInit spawnMessage;
    spawnMessage.mutable_id_()->set_data("vehicle2");
    spawnMessage.set_initlat_(lat2.Degree());
    spawnMessage.set_initlon_(lon2.Degree());
    spawnMessage.set_initz_(depth2);
    spawnMessage.set_initroll_(0.0);
    spawnMessage.set_initpitch_(0.0);
    spawnMessage.set_initheading_(yaw2.Radian());
    spawnMessage.set_acommsaddress_(202);

    spawnPublisher.Publish(spawnMessage);

    Timeout timeout{3s};
    do {
      EXPECT_EQ(1, fixture.Step());
      std::this_thread::sleep_for(100ms);
    } while (observer2.Poses().empty() && !timeout);
  }

  EXPECT_LT(0, observer1.Poses().size());
  EXPECT_LT(0, observer1.SphericalCoordinates().size());
  EXPECT_LT(0, observer2.Poses().size());
  EXPECT_LT(0, observer2.SphericalCoordinates().size());

  std::vector<std::string> topics;
  node.TopicList(topics);

  constexpr double tightTol{1e-5};

  // Check vehicle positions in ENU
  // World origin
  const auto &lastPose1 = observer1.Poses().back();
  EXPECT_NEAR(0.0, lastPose1.Pos().X(), tightTol);
  EXPECT_NEAR(0.0, lastPose1.Pos().Y(), tightTol);
  EXPECT_NEAR(0.0, lastPose1.Pos().Z(), tightTol);
  // Facing North (-90 rotation from default West orientation)
  EXPECT_NEAR(0.0, lastPose1.Rot().Roll(), tightTol);
  EXPECT_NEAR(0.0, lastPose1.Rot().Pitch(), tightTol);
  EXPECT_NEAR(-GZ_PI*0.5, lastPose1.Rot().Yaw(), tightTol);

  const auto &lastLatLon1 = observer1.SphericalCoordinates().back();
  EXPECT_NEAR(lat1.Degree(), lastLatLon1.X(), tightTol);
  EXPECT_NEAR(lon1.Degree(), lastLatLon1.Y(), tightTol);
  EXPECT_NEAR(0.0, lastLatLon1.Z(), tightTol);

  // Higher tolerance for lat/lon because of the conversions
  constexpr double latLonTol{2e-2};

  gz::math::SphericalCoordinates sc;
  sc.SetLatitudeReference(lat1);
  sc.SetLongitudeReference(lon1);
  gz::math::Vector3d expectedPos2 = sc.PositionTransform(
      {lat2.Radian(), lon2.Radian(), 0.0},
      gz::math::SphericalCoordinates::SPHERICAL,
      gz::math::SphericalCoordinates::LOCAL2);
  const auto &lastPose2 = observer2.Poses().back();
  EXPECT_NEAR(expectedPos2.X(), lastPose2.Pos().X(), latLonTol);
  EXPECT_NEAR(expectedPos2.Y(), lastPose2.Pos().Y(), latLonTol);
  EXPECT_NEAR(expectedPos2.Z() - depth2, lastPose2.Pos().Z(), latLonTol);

  const auto &lastLatLon2 = observer2.SphericalCoordinates().back();
  EXPECT_NEAR(lat2.Degree(), lastLatLon2.X(), tightTol);
  EXPECT_NEAR(lon2.Degree(), lastLatLon2.Y(), tightTol);
  EXPECT_NEAR(-depth2, lastLatLon2.Z(), tightTol);

  // For the West-defaulting-vehicle to face South,
  // it has a 90 degree yaw in ENU
  EXPECT_NEAR(0.0, lastPose2.Rot().Roll(), tightTol);
  EXPECT_NEAR(0.0, lastPose2.Rot().Pitch(), tightTol);
  EXPECT_NEAR(GZ_DTOR(90), lastPose2.Rot().Yaw(), tightTol);
}
