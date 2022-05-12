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
TEST(CurrentsTest, TestGlobalCurrent)
{
  TestFixtureWithVehicle fixture(
      "buoyant_tethys_at_depth.sdf", "tethys");
  ignition::transport::Node node;
  auto pub = node.Advertise<ignition::msgs::Vector3d>("/ocean_current");
  ignition::msgs::Vector3d vec;
  vec.set_x(0.0);
  vec.set_y(1.0);
  vec.set_z(0.0);
  pub.Publish(vec);
  fixture.Step(2min);

  const auto &observer = fixture.VehicleObserver();
  const auto &linearVelocities = observer.LinearVelocities();

  // Should be moving in the direction of the current
  EXPECT_NEAR(linearVelocities.back().X(), 0.0, 1e-2);
  EXPECT_NEAR(linearVelocities.back().Y(), 1.0, 1e-1);
  EXPECT_NEAR(linearVelocities.back().Z(), 0.0, 1e-2);
}
