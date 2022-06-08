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

#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include "lrauv_system_tests/TestFixture.hh"

#include "TestConstants.hh"

using namespace lrauv_system_tests;

/// This test checks that the vehicle is neutrally buoyant.
/// It starts the vehicle out in a non-moving fluid and expects zero motion.
TEST(HydrostaticStability, NeutralBuoyancy)
{
  TestFixtureWithVehicle fixture(
    "flat_tethys_no_damping.sdf", "tethys");
  fixture.Step(10000u);

  auto &observer = fixture.VehicleObserver();
  const auto &poses = observer.Poses();
  ASSERT_EQ(10000u, poses.size());
  const gz::math::Pose3d identity;
  EXPECT_EQ(poses[0], identity);
  for (size_t i = 1; i < poses.size(); ++i)
  {
    EXPECT_EQ(poses[i], poses[i - 1]);
  }
}

/// This test checks that the vehicle has a restoring moment. We do this by
/// starting the vehicle of in a world without hydrodynamic damping in a
/// tilted position. Since the vehicle is tilted, restoring moments should
/// kick in causing the vehicle to oscillate. In an actual setting we would
/// also have hydrodynamics which would damp the oscillations and bring the
/// boat to an equilibrium.
TEST(HydrostaticStability, RestoringMoment)
{
  TestFixtureWithVehicle fixture(
    "tilted_tethys_no_damping.sdf", "tethys");

  fixture.Step(10000u);

  auto &observer = fixture.VehicleObserver();
  const auto &poses = observer.Poses();
  ASSERT_EQ(10000u, poses.size());
  double maxPitch = std::abs(poses[0].Rot().Euler().Y());
  for (size_t i = 1; i < poses.size(); ++i)
  {
    EXPECT_NEAR(poses[i - 1].Pos().X(), poses[i].Pos().X(), 1e-2);
    EXPECT_NEAR(poses[i - 1].Pos().Y(), poses[i].Pos().Y(), 1e-2);

    // Oscillations mean the Z value is not exactly constant.
    EXPECT_NEAR(poses[i - 1].Pos().Z(), poses[i].Pos().Z(), 1e-1);

    EXPECT_NEAR(poses[i - 1].Rot().X(), poses[i].Rot().X(), 1e-2);
    EXPECT_NEAR(poses[i - 1].Rot().Z(), poses[i].Rot().Z(), 1e-2);

    maxPitch = std::max(std::abs(poses[i].Rot().Euler().Y()), maxPitch);
  }
  // Since we start the system at 0.08 pitch, we should not exceed this.
  EXPECT_LE(maxPitch, 0.08);
}
