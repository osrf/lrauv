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

#include <ignition/common/Filesystem.hh>

#include "helper/LrauvTestFixture.hh"

//////////////////////////////////////////////////
TEST_F(LrauvTestFixture, DepthVBS)
{
  // Reduce terminal output
  ignition::common::Console::SetVerbosity(3);

  ignition::common::chdir(std::string(LRAUV_APP_PATH));

  // Get initial X
  this->fixture->Server()->Run(true, 10, false);
  EXPECT_EQ(10, this->iterations);
  EXPECT_EQ(10, this->tethysPoses.size());
  EXPECT_NEAR(0.0, this->tethysPoses.back().Pos().X(), 1e-6);

  // Run non blocking
  this->fixture->Server()->Run(false, 0, false);

  // Launch mission
  std::atomic<bool> lrauvRunning{true};
  std::thread lrauvThread([&]()
  {
    LrauvTestFixture::ExecLRAUV(
        "/Missions/RegressionTests/IgnitionTests/testDepthVBS.xml",
        lrauvRunning);
  });

  // Run enough iterations (chosen empirically) to reach steady state, then kill
  // the controller
  int targetIterations{50000};
  int maxSleep{100};
  int sleep{0};
  for (; sleep < maxSleep && lrauvRunning && this->iterations < targetIterations; ++sleep)
  {
    igndbg << "Ran [" << this->iterations << "] iterations." << std::endl;
    std::this_thread::sleep_for(1s);
  }
  EXPECT_LT(sleep, maxSleep);
  ASSERT_LT(targetIterations, this->tethysPoses.size());

  LrauvTestFixture::KillLRAUV();
  lrauvThread.join();

  ignmsg << "Logged [" << this->tethysPoses.size() << "] poses" << std::endl;

  int maxIterations{28000};
  ASSERT_LT(maxIterations, this->tethysPoses.size());

  ignition::math::Vector3d maxVel(0, 0, 0);
  for (int i = 1; i <= this->tethysPoses.size(); i ++)
  {
    // Vehicle roll should be constant
    EXPECT_NEAR(tethysPoses[i].Rot().Euler().X(), 0, 1e-2);

    // Vehicle should not go vertical
    EXPECT_LT(tethysPoses[i].Rot().Euler().Y(), IGN_DTOR(40));

    // Vehicle should not go vertical
    EXPECT_GT(tethysPoses[i].Rot().Euler().Y(), IGN_DTOR(-40));

    // Vehicle should not exceed 20m depth
    // Note: Although the mission has a target depth of 10m, the vehicle has
    // a tendency to overshoot first. Eventually the vehicle will reach steady
    // state.
    EXPECT_GT(tethysPoses[i].Pos().Z(), -20);

    if (tethysLinearVel[i].Length() > maxVel.Length())
    {
      maxVel = tethysLinearVel[i];
    }
  }

  // Vehicle's final pose should be near the 10m mark
  EXPECT_NEAR(tethysPoses.back().Pos().Z(), -10, 1e-1);

  // Vehicle should have almost zero Z velocity at the end.
  EXPECT_NEAR(tethysLinearVel.back().Z(), 0, 1e-1);

  // Expect velocity to approach zero. At the end of the test, the vehicle may
  // not have actually reached zero as it may still be translating or yawing,
  // but its velocity should be less than the maximum velocity/
  EXPECT_LT(tethysLinearVel.back().Length(), maxVel.Length());

  // Vehicle should pitch backward slightly
  // TODO(arjo): enable pitch check after #89 is merged
  // EXPECT_GE(tethysPoses.back().Rot().Euler().Y(), 0);
  // EXPECT_LE(tethysPoses.back().Rot().Euler().Y(), IGN_DTOR(25));
}

