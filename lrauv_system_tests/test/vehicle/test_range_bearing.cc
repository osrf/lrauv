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
#include <future>

#include <lrauv_ignition_plugins/lrauv_command.pb.h>

#include "lrauv_system_tests/RangeBearingClient.hh"
#include "lrauv_system_tests/TestFixture.hh"
#include "lrauv_system_tests/Util.hh"

using namespace lrauv_system_tests;
using namespace std::literals::chrono_literals;

//////////////////////////////////////////////////
TEST(RangeBearingTest, BearingIsCorrect)
{
  // This world has the robot start at a certain depth
  // and 3 acoustic comms nodes
  TestFixture fixture("acoustic_comms_fixture.sdf");
  // Needs to have the server call configure on
  // the RangeBearing plugin before any attempt
  // to request an estimate.
  fixture.Step();

  ignition::transport::Node node;
  RangeBearingClient client(node, "tethys");
  {
    auto future = client.RequestRange(2);
    fixture.Step(50u);
    auto status = future.wait_for(5s);
    ASSERT_EQ(std::future_status::ready, status);

    // Acoustic comms node #2 position w.r.t. the
    // vehicle's ENU local frame is (x, y, z) = (10, 0, 0).
    // The node is directly in front of the vehicle,
    // hence (range, elevation, azimuth) = (10, 0, 0).
    auto commsNode2 = future.get();
    EXPECT_NEAR(commsNode2.bearing().x(), 10., 1e-3);
    EXPECT_NEAR(commsNode2.bearing().y(), 0., 1e-3);
    EXPECT_NEAR(commsNode2.bearing().z(), 0., 1e-3);
  }

  {
    auto future = client.RequestRange(3);
    fixture.Step(50u);
    auto status = future.wait_for(5s);
    ASSERT_EQ(std::future_status::ready, status);

    // Acoustic comms node #3 position w.r.t. the
    // vehicle's ENU local frame is (x, y, z) = (0, -10, 0).
    // The node is directly in front of the vehicle,
    // hence (range, elevation, azimuth) = (10, 0, 1.57).
    auto commsNode3 = future.get();
    EXPECT_NEAR(commsNode3.bearing().x(), 10, 1e-3);
    EXPECT_NEAR(commsNode3.bearing().y(), 0, 1e-3);
    EXPECT_NEAR(commsNode3.bearing().z(), 1.57, 1e-3);
  }

  {
    auto future = client.RequestRange(4);
    fixture.Step(50u);
    auto status = future.wait_for(5s);
    ASSERT_EQ(std::future_status::ready, status);

    // Acoustic comms node #4 position w.r.t. the
    // vehicle's ENU local frame is (x, y, z) = (0, 0, -10).
    // The node is directly in front of the vehicle,
    // hence (range, elevation, azimuth) = (10, 1.57, 0).
    auto commsNode4 = future.get();
    EXPECT_NEAR(commsNode4.bearing().x(), 10, 1e-3);
    EXPECT_NEAR(commsNode4.bearing().y(), 1.57, 1e-3);
    EXPECT_NEAR(commsNode4.bearing().z(), 0, 1e-3);
  }
}
