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

#include <gz/transport/Node.hh>

#include <chrono>
#include <thread>
#include <vector>

#include "lrauv_system_tests/TestFixture.hh"
#include "lrauv_system_tests/ModelObserver.hh"
#include "lrauv_system_tests/Publisher.hh"

using namespace lrauv_system_tests;
using namespace std::literals::chrono_literals;

class HydrodynamicsTestFixture : public TestFixture
{
  public: HydrodynamicsTestFixture() : TestFixture("star_world.sdf")
  {
    for (size_t i = 0; i < 4; ++i)
    {
      this->thrustPublishers.push_back(
          this->node.Advertise<gz::msgs::Double>(
              "/model/tethys" + std::to_string(i + 1) +
              "/joint/propeller_joint/cmd_vel"));
      this->vehicleObservers.push_back(ModelObserver(
          "tethys" + std::to_string(i + 1), "base_link"));
      this->vehicleObservers.back().LimitTo(5s);
    }
  }

  public: std::vector<gz::transport::Node::Publisher> &ThrustPublishers()
  {
    return this->thrustPublishers;
  }

  public: const std::vector<ModelObserver> &VehicleObservers() const
  {
    return this->vehicleObservers;
  }

  protected: void OnPostUpdate(
     const gz::sim::UpdateInfo &_info,
     const gz::sim::EntityComponentManager &_ecm) override
  {
    for (auto &observer : this->vehicleObservers)
    {
      observer.Update(_info, _ecm);
    }
  }

  private: gz::transport::Node node;

  private: std::vector<gz::transport::Node::Publisher> thrustPublishers;

  private: std::vector<ModelObserver> vehicleObservers;
};

template <typename T>
inline std::vector<T> Diff(const std::deque<T> &_input)
{
  std::vector<T> output;
  output.reserve(_input.size() - 1);
  for (size_t i = 1; i < _input.size(); ++i)
  {
    output.push_back(_input[i] - _input[i - 1]);
  }
  return output;
}

/// This test evaluates whether the hydrodynamic plugin successfully performs
/// damping when a thrust is applied.
///
/// Based on previous discussion with MBARI at 300rpm, we should be moving at
/// 1ms^-1. This test checks this behaviour. Furthermore, by starting 4 vehicles
/// out in different positions and orientations, this test makes sure that the
/// transforms of the hydrodynamics plugin are correct.
TEST(HydrodynamicsTest, DampForwardThrust)
{
  HydrodynamicsTestFixture fixture;

  // Step once for simulation to be setup
  fixture.Step();

  gz::msgs::Double thrustCommand;
  // Vehicle is supposed to move at around 1 m/s with a 300 RPM thrust.
  // 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  thrustCommand.set_data(10. * GZ_PI);
  for (auto &publisher : fixture.ThrustPublishers())
  {
    using namespace std::literals::chrono_literals;
    ASSERT_TRUE(WaitForConnections(publisher, 2s));
    publisher.Publish(thrustCommand);
  }

  EXPECT_LT(0, fixture.Step(45s));

  for (const auto &observer : fixture.VehicleObservers())
  {
    const auto &linearVelocities = observer.LinearVelocities();
    EXPECT_LT(0, linearVelocities.size());

    // Expect all final speeds to be similar, around 1m/s as specified early on.
    // TODO: this value seems a little off, possibly due to the sinking motion
    EXPECT_NEAR(linearVelocities.back().Length(), 1.01, 1e-1);

    // Should not have a Z velocity.
    // TODO(arjo): We seem to have a very slight 0.3mm/s sinking motion
    EXPECT_NEAR(linearVelocities.back().Z(), 0, 1e-3);

    // Acceleration should always be decreasing.
    const auto linearAccelerations = Diff(linearVelocities);
    for (size_t i = 1; i < linearAccelerations.size(); ++i)
    {
      EXPECT_LE(linearAccelerations[i].Length(),
                linearAccelerations[i - 1].Length());
    }

    const auto &poses = observer.Poses();

    // Rotations should not have changed much through the course of the test
    EXPECT_EQ(poses.front().Rot(), poses.back().Rot());
  }
}
