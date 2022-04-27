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
  auto &observer = fixture.VehicleObserver();
  // Limit window sizes ensuring overlap between loop iterations
  observer.LimitTo(2min + 10s);

  fixture.Step();
  const auto &poses = observer.Poses();
  const auto &times = observer.Times();
  const auto &linearVelocities = observer.LinearVelocities();
  const auto &angularVelocities = observer.AngularVelocities();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);

  // Launch mission
  //   Mass.position: default
  //   Buouancy.position: neutral
  //   Point.rudderAngle: 9 deg
  //   SetSpeed.speed: 1 m/s
  //   DepthEnvelope.minDepth: 2 m
  //   DepthEnvelope.maxDepth: 20 m
  //   DepthEnvelope.downPitch: -20 deg
  //   DepthEnvelope.upPitch: 20 deg
  auto controller = LRAUVController::Execute({
    "run RegressionTests/IgnitionTests/testYoYoCircle.xml quitAtEnd"});

  for (size_t _ = 0; _ < 5; ++_)
  {
    EXPECT_LT(0, fixture.Step(2min));
    for (size_t i = 0; i < times.size(); ++i)
    {
      // Pitch should be between -20 and 20 degrees
      EXPECT_LT(IGN_DTOR(-20), poses[i].Rot().Pitch());
      EXPECT_GT(IGN_DTOR(20), poses[i].Rot().Pitch());

      if (times[i] > 2min)
      {
        // Check that the vehicle is actually moving
        EXPECT_LT(0.0, linearVelocities[i].Length());
      }

      // Depth should be less than 20m, with some tolerance
      // to accommodate vertical control overshoot
      EXPECT_LT(-20.0 - 4.0, poses[i].Pos().Z());
      if (times[i] > 3min)
      {
        // Depth should be at greater than 2m after initial descent,
        // with some tolerance to accommodate vertical control overshoot
        EXPECT_GT(-2.0 + 1.8, poses[i].Pos().Z());
      }

      if (times[i] > 5min)
      {
        // Once the vehicle achieves its full velocity the vehicle should have
        // a nominal yaw rate of around 0.037-0.038rad/s. This means that the
        // vehicle should keep spinning in a circle.
        EXPECT_NEAR(angularVelocities[i].Z(), 0.037, 0.0022)
            << i << " yaw rate: " << angularVelocities[i].Z();

        // At the same time the roll rate should be near zero
        EXPECT_NEAR(angularVelocities[i].Y(), 0, 1e-1);

        // And the linear velocity should be near 1m/s
        EXPECT_NEAR(linearVelocities[i].Length(), 1.0, 2e-1);
      }
    }
  }
  EXPECT_TRUE(controller.Kill(SIGINT));
  EXPECT_EQ(SIGINT, controller.Wait());
}
