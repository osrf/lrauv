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

using namespace lrauv_system_tests;

//////////////////////////////////////////////////
TEST(BuoyancyActionTest, Sink)
{
  VehicleCommandTestFixture fixture("buoyant_tethys.sdf", "tethys");
  EXPECT_EQ(100u, fixture.Step(100u));

  // Vehicle is at the surface
  const auto &poses = fixture.VehicleObserver().Poses();
  EXPECT_NEAR(-0.5, poses.back().Pos().Z(), 0.05);

  // Command the vehicle to sink
  lrauv_gazebo_plugins::msgs::LRAUVCommand command;
  command.set_dropweightstate_(true);
  command.set_buoyancyaction_(0.0);

  // Step simulation until the command is processed
  // and the vehicle sinks to a certain depth
  constexpr double targetZ{-5.0};
  constexpr uint64_t maxIterations{50000u};
  auto & publisher = fixture.CommandPublisher();
  uint64_t iterations = fixture.Iterations();
  do {
    publisher.Publish(command);
    fixture.Step(400u);
  } while (poses.back().Pos().Z() > targetZ &&
           fixture.Iterations() < maxIterations);

  // Ensure the simulation was stepped forward
  EXPECT_LT(iterations, fixture.Iterations());
  EXPECT_GT(maxIterations, fixture.Iterations());

  // The vehicle sinks
  EXPECT_GT(targetZ, poses.back().Pos().Z());

  // It pitches (world roll) backwards because
  // the buoyancy engine is closer to the back
  EXPECT_NEAR(0.0, poses.back().Rot().Roll(), 0.1);

  // Roll and yaw (in world frame) aren't affected
  EXPECT_NEAR(0.0, poses.back().Rot().Pitch(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Rot().Yaw(), 1e-6);

  // TODO(anyone) Fix residual movement
  // https://github.com/osrf/lrauv/issues/47
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 2.5);
  EXPECT_NEAR(0.0, poses.back().Pos().Y(), 1.0);
}
