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
  // Mass.position: default
  // Buouancy.position: neutral
  // Point.rudderAngle: 9 deg
  // SetSpeed.speed: 1 m/s
  // DepthEnvelope.minDepth: 2 m
  // DepthEnvelope.maxDepth: 20 m
  // DepthEnvelope.downPitch: -20 deg
  // DepthEnvelope.upPitch: 20 deg
  std::atomic<bool> lrauvRunning{true};
  std::thread lrauvThread([&]()
  {
    LrauvTestFixture::ExecLRAUV(
        "/Missions/RegressionTests/IgnitionTests/testYoYoCircle.xml",
        "testYoYoCircle",
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

  int minIterations{28000};
  ASSERT_LT(minIterations, this->tethysPoses.size());

  // Check bounds
  double dtSec = std::chrono::duration<double>(this->dt).count();
  ASSERT_LT(0.0, dtSec);
  double time100it = 100 * dtSec;
  for (unsigned int i = 100; i < this->tethysPoses.size(); i += 100)
  {
    auto prevPose = this->tethysPoses[i - 100];
    auto pose = this->tethysPoses[i];

    // Speed is about 1 m / s
    auto dist = (pose.Pos() - prevPose.Pos()).Length();

    auto linVel = dist / time100it;
    EXPECT_LT(0.0, linVel);

    EXPECT_NEAR(1.0, linVel, 1.0) << i;

    // Depth is above 20m, and below 2m after initial descent, with some
    // tolerance
    EXPECT_LT(-22.5, pose.Pos().Z()) << i;
    if (i > 2000)
    {
      EXPECT_GT(0.3, pose.Pos().Z()) << i;
    }

    // Pitch is between -20 and 20 deg
    EXPECT_LT(IGN_DTOR(-20), pose.Rot().Pitch()) << i;
    EXPECT_GT(IGN_DTOR(20), pose.Rot().Pitch()) << i;

    // Trajectory projected onto the horizontal plane is roughly a circle
    ignition::math::Vector2d planarPos{pose.Pos().X(), pose.Pos().Y()};

    ignition::math::Vector2d center{-4.0, -23.0};
    planarPos -= center;

    double meanRadius{20.0};
    EXPECT_NEAR(20.0, planarPos.Length(), 4.0) << i;
  }
}

