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

#include <chrono>
#include <gtest/gtest.h>

#include "helper/LrauvTestFixture.hh"
#include "lrauv_command.pb.h"
#include "lrauv_state.pb.h"

//////////////////////////////////////////////////
TEST_F(LrauvTestFixture, State)
{
  // TODO(chapulina) Test other fields, see
  // https://github.com/osrf/lrauv/pull/81

  // Initial state
  this->fixture->Server()->Run(true, 100, false);
  int maxSleep{100};
  int sleep{0};
  for (; sleep < maxSleep && this->stateMsgs.size() < 100; ++sleep)
  {
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_EQ(100, this->stateMsgs.size());

  auto latest = this->stateMsgs.back();
  EXPECT_NEAR(0.0, latest.propomega_(), 1e-6);

  // Propel vehicle forward by giving the propeller a positive angular velocity
  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_propomegaaction_(10 * IGN_PI);

  // Neutral buoyancy
  cmdMsg.set_buoyancyaction_(0.0005);
  cmdMsg.set_dropweightstate_(true);

  // Run server until we collect more states
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->stateMsgs.size() < 2000;
  });

  latest = this->stateMsgs.back();
  EXPECT_NEAR(10.0 * IGN_PI, latest.propomega_(), 1e-6);
}

