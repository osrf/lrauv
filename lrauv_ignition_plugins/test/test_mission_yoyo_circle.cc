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

  // Check bounds
  double dtSec = std::chrono::duration_cast<std::chrono::nanoseconds>(
      this->dt).count() * 1e-9;
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

    // Depth is above 20m, with some tolerance
    EXPECT_LT(-22.1, pose.Pos().Z()) << i;
    // Depth is below 2m after initial descent, with some tolerance
    if (i > 2000)
    {
      EXPECT_GT(0.0, pose.Pos().Z()) << i;
    }

    // Pitch is between -20 and 20 deg
    EXPECT_LT(IGN_DTOR(-20), pose.Rot().Pitch()) << i;
    EXPECT_GT(IGN_DTOR(20), pose.Rot().Pitch()) << i;
  }

  // Uncomment to get new expectations
  // for (int i = 500; i <= maxIterations; i += 500)
  // {
  //   auto pose = this->tethysPoses[i];
  //   std::cout << "this->CheckRange(" << i << ", {"
  //       << std::setprecision(2) << std::fixed
  //       << pose.Pos().X() << ", "
  //       << pose.Pos().Y() << ", "
  //       << pose.Pos().Z() << ", "
  //       << pose.Rot().Roll() << ", "
  //       << pose.Rot().Pitch() << ", "
  //       << pose.Rot().Yaw() << "}, {5.0, 0.5});"
  //       << std::endl;
  // }

  // TODO(louise) Make expectations tighter as various inputs are updated

  //          Y
  //            ^
  //            |
  //            |         View from the top
  //            |
  //            |
  //            |     _
  //            |    / |
  //       ,--------'  '_
  //      (             _+ -----------> X
  //       `--------,  ,
  //                 \_|
  //
  // * (X/Y): With a positive rudder angle, the robot rotates counter-clockwise.
  //          So it stays on the -Y side and goes:
  //          -X-Y -> +X-Y -> +X+Y -> -X+Y -> repeat
  // * (Z): starts at zero, but once it goes below -2 m, it never goes deeper than
  //        -20 m, and never higher than -2 m.
  // * (Pitch): always between -20 deg and 20 deg

  // With 0.02 step size, the vehicle does a lap every 6500 iterations or so

  // -X: decrease X
  // -Y: decrease Y
  this->CheckRange(500, {-0.02, -0.00, -0.04, 0.01, -0.00, 0.00}, {1.5, 0.1});
  this->CheckRange(1000, {-3.32, 0.10, 0.07, 0.01, -0.04, 0.14}, {1.5, 0.1});
  this->CheckRange(1500, {-10.77, -1.49, -0.01, 0.00, -0.12, 0.50}, {1.5, 0.1});
  this->CheckRange(2000, {-18.65, -7.10, -0.91, -0.00, -0.20, 0.96}, {1.5, 0.1});
  this->CheckRange(2500, {-23.30, -16.36, -2.58, -0.01, -0.25, 1.47}, {1.5, 0.1});
  this->CheckRange(3000, {-22.74, -26.93, -4.67, -0.01, -0.26, 2.00}, {1.5, 0.1});

  // +X: increase X
  this->CheckRange(3500, {-16.86, -35.83, -6.90, -0.01, -0.27, 2.53}, {1.5, 0.1});
  this->CheckRange(4000, {-7.25, -40.48, -9.22, -0.01, -0.28, 3.07}, {1.5, 0.1});
  this->CheckRange(4500, {3.34, -39.56, -11.74, -0.00, -0.30, -2.67}, {1.5, 0.1});
  this->CheckRange(5000, {11.92, -33.34, -14.37, 0.00, -0.31, -2.13}, {1.5, 0.1});

  // +Y: increase Y
  this->CheckRange(5500, {16.06, -23.60, -17.05, 0.01, -0.31, -1.59}, {1.5, 0.1});
  this->CheckRange(6000, {14.60, -13.17, -19.94, 0.01, -0.18, -1.05}, {1.5, 0.1});

  // -X: decrease X
  this->CheckRange(6500, {7.79, -4.72, -21.49, 0.01, -0.03, -0.52}, {1.5, 0.1});
  this->CheckRange(7000, {-2.51, -0.79, -21.89, 0.01, 0.03, 0.01}, {1.5, 0.1});

  // -Y: decrease Y
  this->CheckRange(7500, {-13.42, -2.58, -21.70, 0.00, 0.08, 0.54}, {1.5, 0.1});
  this->CheckRange(8000, {-21.91, -9.62, -21.05, -0.01, 0.12, 1.07}, {1.5, 0.1});
  this->CheckRange(8500, {-25.66, -19.95, -20.02, -0.01, 0.16, 1.60}, {1.5, 0.1});
  this->CheckRange(9000, {-23.68, -30.70, -18.68, -0.01, 0.19, 2.13}, {1.5, 0.1});

  // +X: increase X
  this->CheckRange(9500, {-16.54, -38.91, -17.07, -0.01, 0.21, 2.66}, {1.5, 0.1});
  this->CheckRange(10000, {-6.28, -42.34, -15.21, -0.00, 0.24, -3.09}, {1.5, 0.1});
  this->CheckRange(10500, {4.25, -40.06, -13.19, 0.01, 0.25, -2.55}, {1.5, 0.1});

  // +Y: increase Y
  this->CheckRange(11000, {12.11, -32.76, -10.96, 0.01, 0.27, -2.01}, {1.5, 0.1});
  this->CheckRange(11500, {15.10, -22.51, -8.59, 0.02, 0.28, -1.47}, {1.5, 0.1});

  // -X: decrease X
  this->CheckRange(12000, {12.41, -12.18, -6.17, 0.02, 0.29, -0.93}, {1.5, 0.1});
  this->CheckRange(12500, {4.84, -4.75, -3.58, 0.01, 0.31, -0.39}, {1.5, 0.1});
  this->CheckRange(13000, {-5.49, -2.28, -1.05, 0.00, 0.08, 0.14}, {1.5, 0.1});

  // -Y: decrease Y
  this->CheckRange(13500, {-15.99, -5.47, -0.26, -0.01, -0.01, 0.67}, {1.5, 0.1});
  this->CheckRange(14000, {-23.50, -13.58, -0.27, -0.01, -0.07, 1.20}, {1.5, 0.1});
  this->CheckRange(14500, {-25.90, -24.35, -0.76, -0.01, -0.11, 1.73}, {1.5, 0.1});

  // +X: increase X
  this->CheckRange(15000, {-22.54, -34.83, -1.65, -0.01, -0.14, 2.26}, {1.5, 0.1});
  this->CheckRange(15500, {-14.38, -42.13, -2.87, 0.00, -0.18, 2.79}, {1.5, 0.1});
  this->CheckRange(16000, {-3.69, -44.27, -4.39, 0.01, -0.21, -2.96}, {1.5, 0.1});

  // +Y: increase Y
  this->CheckRange(16500, {6.54, -40.71, -6.18, 0.02, -0.23, -2.43}, {1.5, 0.1});
  this->CheckRange(17000, {13.51, -32.46, -8.14, 0.01, -0.25, -1.89}, {1.5, 0.1});
  this->CheckRange(17500, {15.27, -21.87, -10.31, 0.01, -0.26, -1.36}, {1.5, 0.1});

  // -X: decrease X
  this->CheckRange(18000, {11.38, -11.89, -12.60, 0.00, -0.27, -0.82}, {1.5, 0.1});
  this->CheckRange(18500, {2.96, -5.35, -15.03, -0.00, -0.29, -0.28}, {1.5, 0.1});
  this->CheckRange(19000, {-7.58, -4.07, -17.61, -0.01, -0.30, 0.26}, {1.5, 0.1});

  // -Y: decrease Y
  this->CheckRange(19500, {-17.23, -8.38, -20.39, -0.02, -0.14, 0.80}, {1.5, 0.1});
  this->CheckRange(20000, {-23.52, -17.30, -21.57, -0.02, -0.01, 1.33}, {1.5, 0.1});
  this->CheckRange(20500, {-24.49, -28.30, -21.76, -0.01, 0.05, 1.86}, {1.5, 0.1});

  // +X: increase X
  this->CheckRange(21000, {-19.78, -38.29, -21.40, 0.00, 0.10, 2.38}, {1.5, 0.1});
  this->CheckRange(21500, {-10.69, -44.51, -20.62, 0.01, 0.14, 2.92}, {1.5, 0.1});
  this->CheckRange(22000, {0.25, -45.28, -19.48, 0.02, 0.17, -2.84}, {1.5, 0.1});

  // +Y: increase Y
  this->CheckRange(22500, {10.02, -40.43, -18.04, 0.02, 0.20, -2.30}, {1.5, 0.1});
  this->CheckRange(23000, {15.94, -31.33, -16.32, 0.01, 0.22, -1.77}, {1.5, 0.1});
  this->CheckRange(23500, {16.40, -20.53, -14.41, -0.00, 0.24, -1.23}, {1.5, 0.1});

  // -X: decrease X
  this->CheckRange(24000, {11.31, -11.06, -12.27, -0.01, 0.26, -0.70}, {1.5, 0.1});
  this->CheckRange(24500, {2.12, -5.54, -10.03, -0.02, 0.27, -0.16}, {1.5, 0.1});

  // -Y: decrease Y
  this->CheckRange(25000, {-8.55, -5.52, -7.61, -0.02, 0.29, 0.38}, {1.5, 0.1});
  this->CheckRange(25500, {-17.68, -10.96, -5.07, -0.02, 0.30, 0.92}, {1.5, 0.1});
  this->CheckRange(26000, {-22.69, -20.27, -2.31, -0.01, 0.20, 1.46}, {1.5, 0.1});

  // +X: increase X
  this->CheckRange(26500, {-22.18, -31.09, -0.67, -0.00, 0.03, 1.99}, {1.5, 0.1});
  this->CheckRange(27000, {-16.19, -40.34, -0.27, 0.01, -0.04, 2.52}, {1.5, 0.1});
  this->CheckRange(27500, {-6.33, -45.33, -0.49, 0.02, -0.09, 3.05}, {1.5, 0.1});

  // +Y: increase Y
  this->CheckRange(28000, {4.68, -44.65, -1.17, 0.02, -0.13, -2.71}, {1.5, 0.1});
}

