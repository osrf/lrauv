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
  EXPECT_LT(maxIterations, this->tethysPoses.size());

  // Give it some iterations to reach steady state
  for (auto i = 2000u; i < this->tethysPoses.size(); i = i + 1000)
  {
    this->CheckRange(i,
      // Target pose has no translation, roll or yaw, and 20 deg pitch
      {0.0, 0.0, 0.0, 0.0, IGN_DTOR(20), 0},
      // Y is kept pretty stable close to zero with low tolerances
      // \TODO(chapulina) X and Y are meant to stay close to zero, but the
      // vehicle goes forward (-X, +Z) slowly. That's caused by the pitch
      // oscillation.
      {4.5, 0.01, 1.6},
      // Roll and yaw are kept pretty stable close to zero with low tolerances
      // \TODO(chapulina) The pitch has a lot of oscillation, so we need the
      // high tolerance
      {IGN_DTOR(2), IGN_DTOR(21), IGN_DTOR(2)});
  }
}

