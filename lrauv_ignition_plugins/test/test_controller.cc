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
  // Vehicle is supposed to move at around 1 m/s with 300 RPM.
  // 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_propomegaaction_(10 * IGN_PI);

  // Neutral buoyancy
  cmdMsg.set_dropweightstate_(true);
  cmdMsg.set_buoyancyaction_(0.0005);

  // Run server until the command is processed and the model reaches a certain
  // point ahead (the robot moves towards -X)
  double targetX{-100.0};
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->tethysPoses.back().Pos().X() > targetX;
  });

  int minIterations{5100};
  EXPECT_LT(minIterations, this->iterations);
  EXPECT_LT(minIterations, this->tethysPoses.size());

  // Check final position
  EXPECT_GT(targetX, this->tethysPoses.back().Pos().X());
  EXPECT_NEAR(0.0, this->tethysPoses.back().Pos().Y(), 1e-3);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Pos().Z(), 0.05);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Roll(), 1e-2);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Pitch(), 1e-3);
  EXPECT_NEAR(0.0, this->tethysPoses.back().Rot().Yaw(), 1e-3);

  // Check approximate instantaneous speed after initial acceleration
  double dtSec = std::chrono::duration<double>(this->dt).count();
  ASSERT_LT(0.0, dtSec);
  double time100it = 100 * dtSec;
  for (unsigned int i = 1800; i < this->tethysPoses.size(); i += 100)
  {
    auto prevPose = this->tethysPoses[i - 100];
    auto pose = this->tethysPoses[i];

    auto dist = (pose.Pos() - prevPose.Pos()).Length();

    auto linVel = dist / time100it;
    EXPECT_LT(0.0, linVel);
    EXPECT_NEAR(1.0, linVel, 0.06) << i;
  }
}

