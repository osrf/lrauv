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

#include <lrauv_gazebo_plugins/lrauv_command.pb.h>
#include "lrauv_system_tests/TestFixture.hh"

#include "TestConstants.hh"

using namespace lrauv_system_tests;

//////////////////////////////////////////////////
TEST(DropWeightTest, ReleaseWeight)
{
  VehicleCommandTestFixture fixture(
      worldPath("buoyant_tethys_at_depth.sdf"), "tethys");
  EXPECT_EQ(100u, fixture.Step(100u));

  // Starting point of the vehicle
  constexpr double startZ{-10};
  // Surface is at 0m
  constexpr double targetZ{0};

  const auto &poses = fixture.VehicleObserver().Poses();
  EXPECT_NEAR(startZ, poses.back().Pos().Z(), 0.05);

  // Tell the vehicle to release the weight
  lrauv_gazebo_plugins::msgs::LRAUVCommand command;
  command.set_dropweightstate_(false);

  // Neutral buoyancy
  command.set_buoyancyaction_(0.0005);

  // Run the server for a fixed number of iterations
  auto & publisher = fixture.CommandPublisher();
  uint64_t iterations = fixture.Iterations();
  for (size_t i = 0; i < 10; ++i)
  {
    publisher.Publish(command);
    fixture.Step(500u);
  }

  // Ensure the simulation was stepped forward
  EXPECT_LT(iterations, fixture.Iterations());

  // Tolerance, the vehicle will breach the surface
  constexpr double tol{0.5};
  for (const auto &pose : poses)
  {
    // Make sure that the vehicle just breaches the surface of the the water
    EXPECT_GT(targetZ + tol, pose.Pos().Z());

    // MAke sure vehicle goes up.
    EXPECT_LT(startZ - tol, pose.Pos().Z());
  }

  // Make sure we do get near the surface
  EXPECT_NEAR(targetZ, poses.back().Pos().Z(), tol);
}
