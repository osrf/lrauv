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

  int maxIterations{28000};
  int maxSleep{100};
  int sleep{0};
  for (; sleep < maxSleep && lrauvRunning && this->iterations < maxIterations; ++sleep)
  {
    igndbg << "Ran [" << this->iterations << "] iterations." << std::endl;
    std::this_thread::sleep_for(1s);
  }
  EXPECT_LT(sleep, maxSleep);
  ASSERT_LT(maxIterations, this->tethysPoses.size());

  LrauvTestFixture::KillLRAUV();
  lrauvThread.join();

  ignmsg << "Logged [" << this->tethysPoses.size() << "] poses" << std::endl;

  // Uncomment to get new expectations
  // for (int i = 2000; i <= maxIterations; i += 2000)
  // {
  //   auto pose = this->tethysPoses[i];
  //   std::cout << "this->CheckRange(" << i << ", {"
  //       << std::setprecision(2) << std::fixed
  //       << pose.Pos().X() << ", "
  //       << pose.Pos().Y() << ", "
  //       << pose.Pos().Z() << ", "
  //       << pose.Rot().Roll() << ", "
  //       << pose.Rot().Pitch() << ", "
  //       << pose.Rot().Yaw() << "}, {12.0, 3.14});"
  //       << std::endl;
  // }

  this->CheckRange(2000, {0.01, -0.00, -0.78, -0.00, 0.04, 0.00}, {15.0, 3.14});
  this->CheckRange(4000, {0.47, -0.00, -2.80, -0.00, 0.08, 0.00}, {15.0, 3.14});
  this->CheckRange(6000, {2.77, -0.00, -5.73, 0.00, 0.12, 0.00}, {15.0, 3.14});
  this->CheckRange(8000, {8.78, -0.04, -9.62, 0.00, 0.12, 0.03}, {15.0, 3.14});
  this->CheckRange(10000, {15.66, 2.84, -13.07, 0.00, 0.09, 1.31}, {15.0, 3.14});
  this->CheckRange(12000, {14.82, 8.71, -15.58, 0.00, 0.04, 2.55}, {15.0, 3.14});
  this->CheckRange(14000, {10.69, 10.22, -16.95, -0.00, -0.03, -2.80}, {15.0, 3.14});
  this->CheckRange(16000, {7.85, 8.85, -16.07, -0.00, -0.01, -2.13}, {15.0, 3.14});
  this->CheckRange(18000, {6.48, 6.29, -13.84, 0.00, -0.07, -1.59}, {15.0, 3.14});
  this->CheckRange(20000, {6.99, 2.26, -10.73, 0.00, -0.17, -0.93}, {15.0, 3.14});
  this->CheckRange(22000, {11.22, -0.97, -7.71, -0.00, -0.10, 0.06}, {15.0, 3.14});
  this->CheckRange(24000, {15.47, 0.32, -5.62, -0.00, 0.04, 0.98}, {15.0, 3.14});
  this->CheckRange(26000, {16.95, 3.13, -5.16, 0.00, 0.00, 1.66}, {15.0, 3.14});
  this->CheckRange(28000, {16.65, 5.57, -6.79, -0.00, 0.02, 2.17}, {15.0, 3.14});
}

