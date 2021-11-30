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

  double totalPitchChange = 0, prevPitch = 0;
  bool firstPitch = false, reachedTarget = false;

  // Vehicle should have a max pitch of 20 degrees
  for(auto pose: this->tethysPoses)
  {
    // Pitch 20 degrees
    ASSERT_LT(pose.Rot().Euler().Y(), IGN_DTOR(21));
    ASSERT_GT(pose.Rot().Euler().Y(), IGN_DTOR(-1));

    // No roll or yaw
    ASSERT_NEAR(pose.Rot().Euler().X(), IGN_DTOR(0), 1e-3);
    ASSERT_NEAR(pose.Rot().Euler().Z(), IGN_DTOR(0), 1e-3);

    // Check position holds
    // TODO(arjo129): Tighten bounds after ignitionrobotics/ign-gazebo#1211 is
    // merged.
    EXPECT_NEAR(pose.Pos().X(), IGN_DTOR(0), 1);
    EXPECT_NEAR(pose.Pos().Y(), IGN_DTOR(0), 1);
    EXPECT_NEAR(pose.Pos().Z(), IGN_DTOR(0), 1);

    // Used later for oscillation check.
    if (firstPitch = false)
    {
      totalPitchChange += std::fabs(pose.Rot().Euler().Y() - prevPitch);
    }

    // Check if we cross the 20 degree mark
    if (prevPitch <= IGN_DTOR(20) && pose.Rot().Euler().Y() >= IGN_DTOR(20))
    {
      reachedTarget = true;
    }

    prevPitch = pose.Rot().Euler().Y();
    firstPitch = true;
  }

  // Check for oscillation by summing over abs delta in pitch
  // Essentially \Sigma abs(f'(x)) < C. In this case C should be near 2*20
  // degrees as the vehicle first pitches down and then comes back up.
  ASSERT_LE(totalPitchChange, IGN_DTOR(21) * 2);

  // Make sure the vehicle actually pitched to 20 degrees.
  ASSERT_TRUE(reachedTarget);
}

