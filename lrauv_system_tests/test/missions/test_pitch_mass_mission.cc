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
TEST(MissionTest, PitchMass)
{
  TestFixtureWithVehicle fixture("buoyant_tethys_at_depth.sdf", "tethys");
  auto &observer = fixture.VehicleObserver();
  // Limit window sizes ensuring overlap between loop iterations
  observer.LimitTo(2min + 10s);

  // Check initial X
  fixture.Step();
  const auto &times = observer.Times();
  const auto &poses = observer.Poses();
  EXPECT_NEAR(0.0, poses.back().Pos().X(), 1e-6);

  // Launch mission
  //   SetSpeed.speed = 0 m/s^2
  //   Point.rudderAngle = 0 deg
  //   Buoyancy.position = neutral
  //   Pitch.pitch = 20 deg
  //   Pitch.elevatorAngle = 0
  auto controller = LRAUVController::Execute({
    // Force neutral buoyancy to prevent the vehicle
    // from surfacing during controller startup
    "configSet VerticalControl.buoyancyDefault 500 cc",
    "run RegressionTests/GazeboTests/testPitchMass.xml quitAtEnd"
  });

  double totalPitchChange = 0, prevPitch = 0;
  bool firstPitch = false, reachedTarget = false;
  for (size_t _ = 0; _ < 5; ++_)
  {
    EXPECT_LT(0u, fixture.Step(2min));

    // Vehicle should have a max pitch of 20 degrees
    // Since the vehicle is facing North (+Y):
    // Euler.Y = roll
    // Euler.X = pitch
    // Euler.Z = yaw
    for (size_t i = 0; i < times.size(); ++i)
    {
      // Check position holds
      EXPECT_NEAR(poses[i].Pos().X(), 0.0, 2e-1);
      EXPECT_NEAR(poses[i].Pos().Y(), 0.0, 2e-1);
      EXPECT_NEAR(poses[i].Pos().Z(), -10.0, 2e-1);

      if (times[i] > 3min)  // Initialization is complete
      {
        // Max Pitch 20 degrees, allowing for an error of
        // up to 5 degrees to accomodate for oscillations
        // during pitch control.
        EXPECT_LT(poses[i].Rot().Euler().X(), GZ_DTOR(25));
        EXPECT_GT(poses[i].Rot().Euler().X(), GZ_DTOR(-5));
      }

      // No roll or yaw
      EXPECT_NEAR(poses[i].Rot().Euler().Y(), GZ_DTOR(0), 1e-2);
      EXPECT_NEAR(poses[i].Rot().Euler().Z(), GZ_DTOR(0), 1e-2);

      // Used later for oscillation check.
      if (!firstPitch)
      {
        totalPitchChange += std::abs(poses[i].Rot().Euler().X() - prevPitch);
        firstPitch = true;
      }

      // Check if we cross the 20 degree mark
      if (prevPitch <= GZ_DTOR(20) && poses[i].Rot().Euler().X() >= GZ_DTOR(20))
      {
        reachedTarget = true;
      }

      prevPitch = poses[i].Rot().Euler().X();
    }
  }
  EXPECT_TRUE(controller.Kill(SIGINT));
  EXPECT_EQ(SIGINT, controller.Wait());

  // Check for oscillation by summing over abs delta in pitch
  // Essentially \Sigma abs(f'(x)) < C. In this case C should be near 2*20
  // degrees as the vehicle first pitches down and then comes back up.
  EXPECT_LE(totalPitchChange, GZ_DTOR(20) * 2.);

  // Make sure the vehicle actually pitched to 20 degrees.
  EXPECT_TRUE(reachedTarget);
}
