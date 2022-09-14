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

#include <gtest/gtest.h>

#include <chrono>

#include <lrauv_gazebo_plugins/lrauv_command.pb.h>
#include "lrauv_system_tests/TestFixture.hh"

using namespace lrauv_system_tests;

//////////////////////////////////////////////////
TEST(ElevatorActionTest, Sink)
{
  VehicleCommandTestFixture fixture(
      "buoyant_tethys.sdf", "tethys");
  uint64_t iterations = fixture.Step(100u);
  EXPECT_EQ(100u, iterations);

  const auto &poses = fixture.VehicleObserver().Poses();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Pos().Y(), 1e-6);
  EXPECT_NEAR(-0.5, poses.back().Pos().Z(), 2e-2);

  // Propel vehicle
  lrauv_gazebo_plugins::msgs::LRAUVCommand command;

  // Move forward
  command.set_propomegaaction_(30);

  // Rotate elevator counter-clockwise when looking from
  // starboard, which causes the vehicle to pitch down.
  // Using an angle lower than the stall angle (0.17)
  command.set_elevatorangleaction_(0.15);

  // Neutral buoyancy
  command.set_buoyancyaction_(0.0005);
  command.set_dropweightstate_(true);

  // Step simulation until the command is processed
  // and the vehicle sinks to a certain depth
  constexpr double targetZ{-1.5};
  constexpr uint64_t maxIterations{50000u};
  auto & publisher = fixture.CommandPublisher();
  do {
    publisher.Publish(command);
    fixture.Step(400u);
  } while (poses.back().Pos().Z() > targetZ &&
           fixture.Iterations() < maxIterations);

  // Ensure the simulation was stepped forward
  EXPECT_LT(iterations, fixture.Iterations());
  EXPECT_GT(maxIterations, fixture.Iterations());

  // Vehicle goes down
  EXPECT_GT(targetZ, poses.back().Pos().Z());

  // Vehicle pitches down
  // NOTE: because the vehicle is starts facing North (+Y in ENU),
  // the nose tilts down with negative roll in the world frame
  // (about the East axis).
  EXPECT_GT(0, poses.back().Rot().Roll());

  // FIXME(chapulina) It goes up a little bit in the beginning,
  // is this expected?
  for (uint64_t i = 0u; i < fixture.Iterations(); ++i)
  {
    EXPECT_GT(0.01, poses[i].Pos().Z()) << "at iteration #" << i;
  }
}
