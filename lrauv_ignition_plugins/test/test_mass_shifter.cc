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
TEST_F(LrauvTestFixture, MassShifterTilt)
{
  // Check initial orientation
  this->fixture->Server()->Run(true, 100, false);
  EXPECT_EQ(100, this->iterations);
  EXPECT_EQ(100, this->tethysPoses.size());
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Roll(), 1e-6);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Pitch(), 1e-6);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Yaw(), 1e-6);

  // Tell the vehicle to tilt downward by moving the mass forward (positive command)
  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_masspositionaction_(0.01);
  cmdMsg.set_buoyancyaction_(0.0005);

  // Run server until the command is processed and the model tilts to a
  // certain angle.
  // Because the vehicle is facing North (+Y in ENU), the nose tilts down with
  // negative "roll" (about East axis)
  double targetRoll{-0.15};
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->tethysPoses.back().Rot().Roll() > targetRoll;
  });

  EXPECT_LT(100, this->iterations);
  EXPECT_LT(100, this->tethysPoses.size());
  EXPECT_GT(targetRoll, this->tethysPoses.back().Rot().Roll());
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Pitch(), 1e-6);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Yaw(), 1e-6);
}

