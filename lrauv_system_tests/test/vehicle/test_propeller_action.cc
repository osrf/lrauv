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

#include <lrauv_ignition_plugins/lrauv_command.pb.h>
#include "lrauv_system_tests/TestFixture.hh"

using namespace lrauv_system_tests;

//////////////////////////////////////////////////
TEST(PropellerActionTest, ForwardThrust)
{
  VehicleCommandTestFixture fixture("buoyant_tethys.sdf", "tethys");
  uint64_t iterations = fixture.Step(100u);
  EXPECT_EQ(100u, iterations);

  // Check initial X
  const auto &poses = fixture.VehicleObserver().Poses();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);

  // Propel vehicle forward by giving the propeller a positive
  // angular velocity. Vehicle is supposed to move at around
  // 1 m/s with 300 RPM. 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  lrauv_ignition_plugins::msgs::LRAUVCommand command;
  command.set_propomegaaction_(10. * GZ_PI);

  // Neutral buoyancy
  command.set_dropweightstate_(true);
  command.set_buoyancyaction_(0.0005);

  // Step simulation until the command is processed and the vehicle
  // travels a certain distance (towards +Y)
  constexpr double targetY{100.0};
  constexpr uint64_t maxIterations{50000u};
  auto & publisher = fixture.CommandPublisher();
  do {
    publisher.Publish(command);
    fixture.Step(100u);
  } while (poses.back().Pos().Y() < targetY &&
           fixture.Iterations() < maxIterations);

  // Ensure the simulation was stepped forward
  EXPECT_LT(iterations, fixture.Iterations());
  EXPECT_GT(maxIterations, fixture.Iterations());

  // Check final position
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-3);
  EXPECT_LT(targetY, poses.back().Pos().Y());
  EXPECT_NEAR(-0.5, poses.back().Pos().Z(), 0.05);
  EXPECT_NEAR(0.0, poses.back().Rot().Roll(), 1e-2);
  EXPECT_NEAR(0.0, poses.back().Rot().Pitch(), 1e-3);
  EXPECT_NEAR(0.0, poses.back().Rot().Yaw(), 1e-3);

  // Check approximate instantaneous speed after initial acceleration
  const auto &linearVelocities = fixture.VehicleObserver().LinearVelocities();
  for (size_t i = 1800u; i < linearVelocities.size(); ++i)
  {
    EXPECT_NEAR(1.0, linearVelocities[i].Length(), 0.06) << i;
  }
}
