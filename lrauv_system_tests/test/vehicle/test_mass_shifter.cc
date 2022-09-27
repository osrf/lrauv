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
TEST(MassShifterTest, DownwardTilt)
{
  VehicleCommandTestFixture fixture(
      worldPath("buoyant_tethys_at_depth.sdf"), "tethys");
  uint64_t iterations = fixture.Step(100u);
  EXPECT_EQ(100u, iterations);

  // Check initial orientation
  const auto &poses = fixture.VehicleObserver().Poses();
  auto lastOrientation = poses.back().Rot();
  EXPECT_NEAR(0.0, lastOrientation.Roll(), 1e-6);
  EXPECT_NEAR(0.0, lastOrientation.Pitch(), 1e-6);
  EXPECT_NEAR(0.0, lastOrientation.Yaw(), 1e-6);

  // Tell the vehicle to tilt downward by moving
  // the mass forward (positive command)
  lrauv_gazebo_plugins::msgs::LRAUVCommand command;
  command.set_masspositionaction_(0.01);
  command.set_buoyancyaction_(0.0005);

  // Step simulation until the command is processed and the vehicle
  // tilts to a certain angle
  // NOTE: because the vehicle is starts facing North (+Y in ENU),
  // the nose tilts down with negative roll in the world frame
  // (about the East axis).
  constexpr double targetRoll{-0.15};
  constexpr uint64_t maxIterations{50000u};
  auto & publisher = fixture.CommandPublisher();
  do {
    publisher.Publish(command);
    fixture.Step(400u);
  } while (poses.back().Rot().Roll() > targetRoll &&
           fixture.Iterations() < maxIterations);

  // Ensure the simulation was stepped forward
  EXPECT_LT(iterations, fixture.Iterations());
  EXPECT_GT(maxIterations, fixture.Iterations());

  lastOrientation = poses.back().Rot();
  EXPECT_GT(targetRoll, lastOrientation.Roll());
  EXPECT_NEAR(0.0, lastOrientation.Pitch(), 1e-6);
  EXPECT_NEAR(0.0, lastOrientation.Yaw(), 1e-6);
}
