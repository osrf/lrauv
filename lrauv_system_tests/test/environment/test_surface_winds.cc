/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "lrauv_system_tests/TestFixture.hh"

#include "TestConstants.hh"

using namespace lrauv_system_tests;
using namespace std::literals::chrono_literals;

//////////////////////////////////////////////////
TEST(SurfaceWindsTest, WindAtSurface)
{
  TestFixtureWithVehicle fixture(
      worldPath("windy_tethys.sdf"), "tethys");
  fixture.Step(2min);
  // Eastward velocity at the surface due to wind
  // NOTE(hidmic): reverse Y-axis velocity sign to match FSK
  const auto &observer = fixture.VehicleObserver();
  const auto &linearVelocities = observer.LinearVelocities();
  EXPECT_LT(1e-3, -linearVelocities.back().Y());
}

//////////////////////////////////////////////////
TEST(SurfaceWindsTest, NoWindAtDepth)
{
  TestFixtureWithVehicle fixture(
      worldPath("windy_tethys_at_depth.sdf"), "tethys");
  fixture.Step(2min);
  // No eastward velocity at a depth due to wind
  const auto &observer = fixture.VehicleObserver();
  const auto &linearVelocities = observer.LinearVelocities();
  EXPECT_NEAR(0.0, linearVelocities.back().Y(), 1e-6);
}
