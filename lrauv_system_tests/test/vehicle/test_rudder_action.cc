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

//////////////////////////////////////////////////
TEST(RudderActionTest, LeftTurn)
{
  using TestFixture = lrauv_system_tests::VehicleCommandTestFixture;
  TestFixture fixture("buoyant_tethys_at_depth.sdf", "tethys");
  uint64_t iterations = fixture.Step(100u);
  EXPECT_EQ(100u, iterations);

  // Check initial position
  const auto &poses = fixture.VehicleObserver().Poses();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Pos().Y(), 1e-6);

  // Propel vehicle
  lrauv_ignition_plugins::msgs::LRAUVCommand command;

  // Move forward
  command.set_propomegaaction_(30);

  // Rotate rudder clockwise when looking from the top,
  // which causes the vehicle to move in a counter-clockwise arch
  command.set_rudderangleaction_(0.8);

  // Neutral buoyancy
  command.set_buoyancyaction_(0.0005);

  // Step simulation until the command is processed and
  // the vehicle reaches a certain point laterally
  constexpr double targetY{-1.5};
  constexpr uint64_t maxIterations{50000u};
  auto & publisher = fixture.CommandPublisher();
  do {
    publisher.Publish(command);
    fixture.Step(400u);
  } while (poses.back().Pos().Y() > targetY &&
           fixture.Iterations() < maxIterations);

  // Ensure the simulation was stepped forward
  EXPECT_LT(iterations, fixture.Iterations());
  EXPECT_GT(maxIterations, fixture.Iterations());

  // Vehicle makes a clockwise arch looking from the top
  EXPECT_GT(targetY, poses.back().Pos().Y());
  EXPECT_GT(-1.0, poses.back().Pos().X());
}
