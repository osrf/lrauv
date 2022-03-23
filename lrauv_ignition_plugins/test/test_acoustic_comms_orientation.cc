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

#include <chrono>
#include <gtest/gtest.h>

#include "helper/LrauvTestFixture.hh"
#include "lrauv_command.pb.h"

#include <fstream>
#include "helper/LrauvRangeBearingClient.hh"

/// \brief Loads the tethys "tethys_acoustic_comms.sdf" world.
/// This world has the robot start at a certain depth and 3 acoustic comms nodes
class LrauvTestCommsOrientation : public LrauvTestFixtureBase
{
  /// Documentation inherited
  protected: void SetUp() override
  {
    LrauvTestFixtureBase::SetUp("tethys_acoustic_comms_test.sdf");
  }
};

//////////////////////////////////////////////////
TEST_F(LrauvTestCommsOrientation, CheckCommsOrientation)
{
  /// Needs to have the server call configure on the plugin before requesting.
  this->fixture->Server()->Run(true, 100, false);
  this->fixture->Server()->Run(false, 10000, false);

  RangeBearingClient client("tethys");
  auto box1 = client.RequestRange(2);
  auto box2 = client.RequestRange(3);
  auto box3 = client.RequestRange(4);

  // Range bearing in FSK frame with elevation being calculated by
  // Box2 cartesian coordinates in vehicle ENU local frame (10, 0, 0)
  // Note convention is (range, elevation, azimuth)
  // Box1 is directly infront of the vehicle hence the bearing is (10, 0, 0)
  EXPECT_NEAR(box1.bearing().x(), 10, 1e-3);
  EXPECT_NEAR(box1.bearing().y(), 0, 1e-3);
  EXPECT_NEAR(box1.bearing().z(), 0, 1e-3);

  // Range bearing in FSK frame
  // Box2 cartesian coordinates in vehicle ENU local frame (0, -10, 0)
  // ?? Seems weird
  // Box2 is to the right of the vehicle hence the bearing is (10, 0, 1.57)
  EXPECT_NEAR(box2.bearing().x(), 10, 1e-3);
  EXPECT_NEAR(box2.bearing().y(), 0, 1e-3);
  EXPECT_NEAR(box2.bearing().z(), 1.57, 1e-3);

  // Range bearing in FSK frame
  // Box3 cartesian coordinates in vehicle ENU local frame (0, 0, -10)
  // Box3 is directly below the vehicle hence the bearing is (10, 1.57, 0)
  EXPECT_NEAR(box3.bearing().x(), 10, 1e-3);
  EXPECT_NEAR(box3.bearing().y(), 1.57, 1e-3);
  EXPECT_NEAR(box3.bearing().z(), 0, 1e-3);
}