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

#include "lrauv_system_tests/LRAUVController.hh"
#include "lrauv_system_tests/TestFixture.hh"

using namespace lrauv_system_tests;
using namespace std::literals::chrono_literals;

//////////////////////////////////////////////////
TEST(MissionTest, YoYoCircle)
{
  TestFixtureWithVehicle fixture("buoyant_tethys.sdf", "tethys");
  fixture.Step(10u);

  const auto &observer = fixture.VehicleObserver();
  const auto &poses = observer.Poses();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);

  {
    // Launch mission
    //   Mass.position: default
    //   Buouancy.position: neutral
    //   Point.rudderAngle: 9 deg
    //   SetSpeed.speed: 1 m/s
    //   DepthEnvelope.minDepth: 2 m
    //   DepthEnvelope.maxDepth: 20 m
    //   DepthEnvelope.downPitch: -20 deg
    //   DepthEnvelope.upPitch: 20 deg
    auto controller = LRAUVController::Execute(
        "RegressionTests/IgnitionTests/testYoYoCircle.xml");

    // Run for a bit longer than mission timeout
    fixture.Step(10min);
  }

  const auto &linearVelocities = observer.LinearVelocities();
  const auto &angularVelocities = observer.AngularVelocities();
  for (uint64_t i = 100u; i < fixture.Iterations(); i += 100u)
  {
    // Pitch should be between -20 and 20 degrees
    EXPECT_LT(IGN_DTOR(-20), poses[i].Rot().Pitch()) << i;
    EXPECT_GT(IGN_DTOR(20), poses[i].Rot().Pitch()) << i;

    if (i > 3500u)
    {
      // Check that the vehicle is actually moving
      EXPECT_LT(0.0, linearVelocities[i].Length()) << i;
    }

    // Depth should be less than 20m at all times
    // EXPECT_LT(-22.7, pose.Pos().Z()) << i;
    EXPECT_LT(-20.0, poses[i].Pos().Z()) << i;
    if (i > 4000u)
    {
      // Depth should be at greater than 2m after initial descent
      EXPECT_GT(-2.0, poses[i].Pos().Z()) << i;
    }

    if (i > 14000u)
    {
      // Once the vehicle achieves its full velocity the vehicle should have
      // a nominal yaw rate of around 0.037-0.038rad/s. This means that the
      // vehicle should keep spinning in a circle.
      EXPECT_NEAR(angularVelocities[i].Z(), 0.037, 0.0022)
        << i << " yaw rate: " << angularVelocities[i].Z();

      // At the same time the roll rate should be near zero
      EXPECT_NEAR(angularVelocities[i].Y(), 0, 1e-1) << i;

      // And the linear velocity should be near 1m/s
      EXPECT_NEAR(linearVelocities[i].Length(), 1.0, 2e-1) << i;
    }
  }
}

