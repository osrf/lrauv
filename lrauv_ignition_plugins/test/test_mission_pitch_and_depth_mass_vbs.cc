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
TEST_F(LrauvTestFixture, PitchDepthVBS)
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
        "/Missions/RegressionTests/IgnitionTests/testPitchAndDepthMassVBS.xml",
        lrauvRunning);
  });

  int maxSleep{100};
  int sleep{0};
  for (; sleep < maxSleep && lrauvRunning; ++sleep)
  {
    igndbg << "Ran [" << this->iterations << "] iterations." << std::endl;
    std::this_thread::sleep_for(1s);
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_FALSE(lrauvRunning);

  lrauvThread.join();

  ignmsg << "Logged [" << this->tethysPoses.size() << "] poses" << std::endl;

  int maxIterations{28000};
  ASSERT_LT(maxIterations, this->tethysPoses.size());

  bool targetReached = false;
  // Vehicle should sink to 10 meters and hold there
  // Pitch should be held relatively constant.
  for (const auto pose: this->tethysPose)
  {
    ASSERT_LT(pose.Pos().Z(), 0.1);
    ASSERT_GT(pose.Pos().Z(), 11.5);

    ASSERT_NEAR(pose.Pos().X(), 0, 1e-1);
    ASSERT_NEAR(pose.Pos().Y(), 0, 1e-1);

    ASSERT_NEAR(pose.Rot().Euler().X(), 0, 1e-1);
    ASSERT_NEAR(pose.Rot().Euler().Y(), 0, 1e-1);
    ASSERT_NEAR(pose.Rot().Euler().Z(), 0, 1e-1);
  }
}