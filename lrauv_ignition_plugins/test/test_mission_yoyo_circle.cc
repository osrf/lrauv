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

#include <chrono>
#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>

#include "helper/LrauvTestFixture.hh"

//////////////////////////////////////////////////
TEST_F(LrauvTestFixture, YoYoCircle)
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
        "/Missions/RegressionTests/IgnitionTests/testYoYoCircle.xml",
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
  //       << pose.Rot().Yaw() << "}, {12.0, 1.57});"
  //       << std::endl;
  // }

  this->CheckRange(2000, {-16.74, 5.18, -0.43, 0.01, -0.16, -0.83}, {12.0, 1.57});
  this->CheckRange(4000, {-14.39, 37.93, -6.73, -0.01, -0.27, -2.69}, {12.0, 1.57});
  this->CheckRange(6000, {15.90, 25.45, -15.35, -0.01, -0.30, 1.68}, {12.0, 1.57});
  this->CheckRange(8000, {-6.70, 1.26, -22.21, 0.02, 0.01, -0.23}, {12.0, 1.57});
  this->CheckRange(10000, {-23.28, 31.18, -20.58, -0.02, 0.15, -2.12}, {12.0, 1.57});
  this->CheckRange(12000, {9.78, 37.26, -14.94, 0.01, 0.24, 2.26}, {12.0, 1.57});
  this->CheckRange(14000, {4.68, 4.65, -7.06, 0.02, 0.30, 0.34}, {12.0, 1.57});
  this->CheckRange(16000, {-24.15, 20.89, -0.08, -0.02, -0.00, -1.56}, {12.0, 1.57});
  this->CheckRange(18000, {1.11, 43.97, -1.60, 0.02, -0.15, 2.82}, {12.0, 1.57});
  this->CheckRange(20000, {14.68, 13.21, -7.16, -0.01, -0.24, 0.92}, {12.0, 1.57});
  this->CheckRange(22000, {-18.23, 10.67, -15.00, -0.03, -0.30, -0.99}, {12.0, 1.57});
  this->CheckRange(24000, {-8.99, 42.41, -21.96, 0.04, 0.00, -2.91}, {12.0, 1.57});
  this->CheckRange(26000, {19.17, 23.00, -20.42, -0.04, 0.15, 1.48}, {12.0, 1.57});
  this->CheckRange(28000, {-7.79, 2.95, -14.81, 0.02, 0.24, -0.43}, {12.0, 1.57});
}

