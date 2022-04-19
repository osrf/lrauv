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

  EXPECT_EQ(10u, fixture.Step(10u));

  // Check initial pose (facing North)
  const auto &observer = fixture.VehicleObserver();
  const auto &poses = observer.Poses();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Pos().Y(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Pos().Z(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Rot().Roll(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Rot().Pitch(), 1e-6);
  EXPECT_NEAR(0.0, poses.back().Rot().Yaw(), 1e-6);

  {
    // Launch mission
    //   SetSpeed.speed = 0 m/s^2
    //   Point.rudderAngle = 0 deg
    //   Pitch.depth = 10 m
    //   Pitch.elevatorAngle = 0
    auto controller = LRAUVController::Execute(
        "RegressionTests/IgnitionTests/testPitchAndDepthMassVBS.xml");

    // Run for a bit longer than mission timeout
    fixture.Step(10min);
  };

  bool targetReached = false, firstSample = true;
  double prevZ = 0, totalDepthChange = 0;
  // Vehicle should sink to 10 meters and hold there
  // Pitch (rotation about world's X, since robot is facing North (+Y)) should
  // be held relatively constant.
  for (const auto &pose : poses)
  {
    // Vehicle should dive down.
    EXPECT_LE(pose.Pos().Z(), 0.5);
    // FIXME(arjo): This should dive to a max of 10m I think
    EXPECT_GT(pose.Pos().Z(), -21.5);

    // Vehicle should exhibit minimal lateral translation.
    EXPECT_NEAR(pose.Pos().X(), 0, 1e-2);
    EXPECT_NEAR(pose.Pos().Y(), 0, 3.0); // FIXME(arjo): IMPORTANT!!

    // Vehicle should hold a fixed angle about X
    // FIXME(arjo): Shouldnt be pitching this much
    EXPECT_NEAR(pose.Rot().Euler().X(), 0, 4e-1);
    EXPECT_NEAR(pose.Rot().Euler().Y(), 0, 1e-2);
    EXPECT_NEAR(pose.Rot().Euler().Z(), 0, 1e-2);

    if (!firstSample)
    {
      // Check we actually crossed the 10m mark
      if (prevZ >= -10 && pose.Pos().Z() <= -10)
      {
        targetReached = true;
      }
      // Use total depth change as a proxy for oscillations
      totalDepthChange += std::fabs(pose.Pos().Z() - prevZ);
    }
    prevZ = pose.Pos().Z();
    firstSample = false;
  }
  // vehicle should have reached the desired target.
  EXPECT_TRUE(targetReached);

  // vehicle should not oscillate when at target.
  // FIXME(arjo): Change this value. It should be 10.
  EXPECT_LT(totalDepthChange, 50);
}
