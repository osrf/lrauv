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

#include <fstream>

//////////////////////////////////////////////////
TEST_F(LrauvTestFixture, Sink)
{
  // Get initial Z
  this->fixture->Server()->Run(true, 100, false);
  EXPECT_EQ(100, this->iterations);
  EXPECT_EQ(100, this->tethysPoses.size());
  EXPECT_NEAR(0.0, this->tethysPoses.back().Pos().Z(), 0.05);

  // Tell the vehicle to sink
  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_dropweightstate_(true);
  cmdMsg.set_buoyancyaction_(0.0);

  // Run server until the command is processed and the model sinks to a
  // certain height
  double targetZ{-5.0};
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->tethysPoses.back().Pos().Z() > targetZ;
  });

  EXPECT_LT(100, this->iterations);
  EXPECT_LT(100, this->tethysPoses.size());

  // The vehicle sinks
  EXPECT_GT(targetZ, this->tethysPoses.back().Pos().Z());

  // It pitches backwards because the buoyancy engine is closer to the back
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Pitch(), 0.14);

  // Roll and yaw aren't affected
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Roll(), 1e-6);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Yaw(), 1e-6);

  // TODO(anyone) Fix residual movement
  // https://github.com/osrf/lrauv/issues/132
  EXPECT_NEAR(0.0, this->tethysPoses.back().Pos().X(), 2.8);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Pos().Y(), 1.1);
}

