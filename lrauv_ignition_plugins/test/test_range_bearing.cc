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

#include "helper/LrauvCommsFixture.hh"
#include "helper/LrauvRangeBearingClient.hh"

using namespace lrauv_ignition_plugins;

TEST_F(LrauvCommsFixture, TestRangingAccuracy)
{
  /// Needs to have the server call configure on the plugin before requesting.
  this->fixture->Server()->Run(true, 100, false);
  this->fixture->Server()->Run(false, 2000, false);

  RangeBearingClient client("box1");
  auto result = client.RequestRange(2);

  EXPECT_EQ(result.req_id(), 2);
  EXPECT_NEAR(result.range(), 20, 0.5);
  EXPECT_NEAR(result.bearing().x(), 20, 1e-3);
  EXPECT_NEAR(result.bearing().y(), 0., 1e-2);
  EXPECT_NEAR(result.bearing().z(), -1.57, 1e-2);
}
