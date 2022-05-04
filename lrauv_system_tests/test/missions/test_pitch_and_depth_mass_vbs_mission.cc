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
TEST(MissionTest, PitchDepthVBS)
{
  TestFixtureWithVehicle fixture("buoyant_tethys.sdf", "tethys");
  auto &observer = fixture.VehicleObserver();
  // Limit window sizes ensuring overlap between loop iterations
  observer.LimitTo(2min + 10s);

  // Check initial pose (facing North)
  fixture.Step();
  const auto &times = observer.Times();
  const auto &poses = observer.Poses();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Pos().Y(), 1e-6);
  EXPECT_NEAR(-0.5, poses.back().Pos().Z(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Rot().Roll(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Rot().Pitch(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Rot().Yaw(), 1e-6);

  // Launch mission
  //   SetSpeed.speed = 0 m/s^2
  //   Point.rudderAngle = 0 deg
  //   Pitch.depth = 10 m
  //   Pitch.elevatorAngle = 0
  auto controller = LRAUVController::Execute({
    "run RegressionTests/IgnitionTests/testPitchAndDepthMassVBS.xml quitAtEnd"});

  bool targetReached = false, firstSample = true;
  double prevZ = 0, totalDepthChange = 0;
  for (size_t _ = 0; _ < 5; ++_)
  {
    EXPECT_LT(0u, fixture.Step(2min));
    for (size_t i = 0; i < times.size(); ++i)
    {
      // Vehicle should not breach the surface.
      EXPECT_LE(poses[i].Pos().Z(), 0.0);
      // NOTE: vehicle target depth is 10m, but vertical
      // control exhibits considerable overshoot.
      EXPECT_GT(poses[i].Pos().Z(), -20.);

      // Vehicle should exhibit minimal lateral translation.
      // FIXME: Hydrodynamic damping and graded buoyancy introduce
      // spurious thrusts. Reduce tolerances when fixed.
      EXPECT_NEAR(poses[i].Pos().X(), 0, 0.3);
      EXPECT_NEAR(poses[i].Pos().Y(), 0, 5.0);

      if (times[i] > 3min)  // Initialization is complete
      {
        // Vehicle should hold a fixed angle about X
        EXPECT_NEAR(poses[i].Rot().Euler().X(), 0.0, 1e-1);
        EXPECT_NEAR(poses[i].Rot().Euler().Y(), 0.0, 1e-2);
        EXPECT_NEAR(poses[i].Rot().Euler().Z(), 0.0, 1e-2);

        if (!firstSample)
        {
          // Check we actually crossed the 10m mark
          if (prevZ >= -10.0 && poses[i].Pos().Z() <= -10.0)
          {
            targetReached = true;
          }
          // Use total depth change as a proxy for oscillations
          totalDepthChange += std::fabs(poses[i].Pos().Z() - prevZ);
        }
        prevZ = poses[i].Pos().Z();
        firstSample = false;
      }
    }
  }
  EXPECT_TRUE(controller.Kill(SIGINT));
  EXPECT_EQ(SIGINT, controller.Wait());

  // Vehicle should have reached the desired target.
  EXPECT_TRUE(targetReached);

  // Vehicle oscillations should be limited.
  EXPECT_LT(totalDepthChange, 30.0);
}
