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
TEST_F(LrauvTestFixture, Command)
{
  // Get initial X
  this->fixture->Server()->Run(true, 100, false);
  EXPECT_EQ(100, this->iterations);
  EXPECT_EQ(100, this->tethysPoses.size());
  EXPECT_NEAR(0.0, this->tethysPoses.back().Pos().X(), 1e-6);

  // Propel vehicle forward by giving the propeller a positive angular velocity
  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_propomegaaction_(30);

  // Neutral buoyancy
  cmdMsg.set_dropweightstate_(true);
  cmdMsg.set_buoyancyaction_(0.0005);

  // Run server until the command is processed and the model reaches a certain
  // point ahead (the robot moves towards -X)
  double targetX{-1.5};
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->tethysPoses.back().Pos().X() > targetX;
  });

  EXPECT_LT(100, this->iterations);
  EXPECT_LT(100, this->tethysPoses.size());
  EXPECT_GT(targetX, this->tethysPoses.back().Pos().X());
}

