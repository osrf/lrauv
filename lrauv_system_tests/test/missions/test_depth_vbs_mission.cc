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
TEST(MissionTest, DepthVBS)
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
    auto controller = LRAUVController::Execute(
        "RegressionTests/IgnitionTests/testDepthVBS.xml");

    // Run for a bit longer than mission timeout
    fixture.Step(20min);
  }

  auto maxLinearVelocity = ignition::math::Vector3d::Zero;
  const auto &linearVelocities = observer.LinearVelocities();
  for (uint64_t i = 10u; i <= fixture.Iterations(); ++i)
  {
    // Vehicle roll (about +Y since vehicle is facing North) should be constant
    EXPECT_NEAR(poses[i].Rot().Euler().Y(), 0, 1e-2);

    // Vehicle should not go vertical when tilting nose
    EXPECT_LT(poses[i].Rot().Euler().X(), IGN_DTOR(40));
    EXPECT_GT(poses[i].Rot().Euler().X(), IGN_DTOR(-40));

    // Vehicle should not exceed 20m depth
    // NOTE: Although the mission has a target depth of 10m, the vehicle has
    // a tendency to overshoot first. Eventually the vehicle will reach steady
    // state, however at this point we are just checking for excess overshoot.
    EXPECT_GT(poses[i].Pos().Z(), -20);

    if (linearVelocities[i].Length() > maxLinearVelocity.Length())
    {
      maxLinearVelocity = linearVelocities[i];
    }
  }

  // FIXME(hidmic): hydrodynamic damping forces induce
  // a thrust on the vehicle, and thus it does not
  // remain steady unless controlled.
  // // Vehicle's final pose should be near the 10m mark
  // EXPECT_NEAR(tethysPoses.back().Pos().Z(), -10, 1e-1);

  // // Vehicle should have almost zero Z velocity at the end.
  // EXPECT_NEAR(tethysLinearVel.back().Z(), 0, 1e-1);

  // // Expect velocity to approach zero. At the end of the test, the vehicle may
  // // not have actually reached zero as it may still be translating or yawing,
  // // but its velocity should be less than the maximum velocity.
  // EXPECT_LT(tethysLinearVel.back().Length(), maxVel.Length());

  // Vehicle should pitch backward slightly
  // TODO(arjo): enable pitch check after #89 is merged
  // EXPECT_GE(tethysPoses.back().Rot().Euler().Y(), 0);
  // EXPECT_LE(tethysPoses.back().Rot().Euler().Y(), IGN_DTOR(25));
}

