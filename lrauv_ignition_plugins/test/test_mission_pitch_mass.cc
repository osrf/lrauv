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
TEST_F(LrauvTestFixture, PitchMass)
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
  // SetSpeed.speed = 0 m/s^2
  // Point.rudderAngle = 0 deg
  // Buoyancy.position = neutral
  // Pitch.pitch = 20 deg
  // Pitch.elevatorAngle = 0
  std::atomic<bool> lrauvRunning{true};
  std::thread lrauvThread([&]()
  {
    LrauvTestFixture::ExecLRAUV(
        "/Missions/RegressionTests/IgnitionTests/testPitchMass.xml",
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

  // * Y, roll and yaw are kept pretty stable close to zero with low tolerances
  // * The pitch target is 20 deg, but there's a lot of oscillation, so we need
  //   the high tolerance
  // * X and Y are meant to stay close to zero, but the vehicle goes forward
  //   (-X, +Z) slowly. That could be caused by the pitch oscillation?
  this->CheckRange(2000,  {-0.01, 0.00, -0.06, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(4000,  {-0.17, 0.00, -0.01, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(6000,  {-0.51, 0.00,  0.12, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(8000,  {-0.87, 0.00,  0.22, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(10000, {-1.21, 0.00,  0.34, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(12000, {-1.55, 0.00,  0.50, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(14000, {-1.90, 0.00,  0.63, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(16000, {-2.25, 0.00,  0.73, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(18000, {-2.60, 0.00,  0.86, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(20000, {-2.93, 0.00,  1.02, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(22000, {-3.29, 0.00,  1.13, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(24000, {-3.64, 0.00,  1.24, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(26000, {-3.97, 0.00,  1.39, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
  this->CheckRange(28000, {-4.32, 0.00,  1.53, 0.00, IGN_DTOR(20), 0.00},
      {0.1, 0.01, 0.1}, {IGN_DTOR(2), IGN_DTOR(18), IGN_DTOR(2)});
}

