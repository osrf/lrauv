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
  //       << pose.Rot().Yaw() << "}, {12.0, 1.57});"
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

  // With 0.02 step size, the vehicle does a lap every 7000 iterations or so

  // -X: decrease X
  // -Y: decrease Y
  this->CheckRange(500, {-0.16, 0.00, -0.04, 0.01, -0.00, 0.00}, {12.0, 1.57});
  this->CheckRange(1000, {-3.61, 0.09, 0.09, 0.01, -0.04, 0.15}, {12.0, 1.57});
  this->CheckRange(1500, {-10.30, -1.30, 0.04, -0.00, -0.11, 0.47}, {12.0, 1.57});
  this->CheckRange(2000, {-17.51, -5.89, -0.58, -0.01, -0.17, 0.88}, {12.0, 1.57});
  this->CheckRange(2500, {-22.54, -13.64, -1.73, -0.01, -0.21, 1.33}, {12.0, 1.57});
  this->CheckRange(3000, {-23.68, -23.05, -3.24, -0.01, -0.24, 1.79}, {12.0, 1.57});

  // +X: increase X
  this->CheckRange(3500, {-20.43, -32.02, -5.03, -0.01, -0.26, 2.27}, {12.0, 1.57});
  this->CheckRange(4000, {-13.43, -38.51, -6.97, -0.00, -0.27, 2.74}, {12.0, 1.57});
  this->CheckRange(4500, {-4.24, -41.06, -9.00, 0.00, -0.29, -3.06}, {12.0, 1.57});
  this->CheckRange(5000, {5.05, -39.10, -11.18, 0.01, -0.29, -2.58}, {12.0, 1.57});

  // +Y: increase Y
  this->CheckRange(5500, {12.38, -33.08, -13.39, 0.02, -0.30, -2.10}, {12.0, 1.57});
  this->CheckRange(6000, {16.10, -24.36, -15.63, 0.02, -0.31, -1.62}, {12.0, 1.57});

  // -X: decrease X
  this->CheckRange(6500, {15.37, -14.96, -17.99, 0.02, -0.32, -1.14}, {12.0, 1.57});
  this->CheckRange(7000, {10.39, -7.01, -20.55, 0.02, -0.16, -0.66}, {12.0, 1.57});
  this->CheckRange(7500, {2.00, -2.06, -21.82, 0.00, -0.04, -0.19}, {12.0, 1.57});
  this->CheckRange(8000, {-7.86, -1.46, -22.27, -0.01, 0.01, 0.28}, {12.0, 1.57});

  // -Y: decrease Y
  this->CheckRange(8500, {-16.95, -5.44, -22.31, -0.02, 0.06, 0.76}, {12.0, 1.57});
  this->CheckRange(9000, {-23.22, -13.11, -21.99, -0.02, 0.10, 1.23}, {12.0, 1.57});
  this->CheckRange(9500, {-25.29, -22.76, -21.37, -0.01, 0.13, 1.70}, {12.0, 1.57});

  // +X: increase X
  this->CheckRange(10000, {-22.75, -32.26, -20.48, 0.01, 0.16, 2.18}, {12.0, 1.57});
  this->CheckRange(10500, {-16.18, -39.51, -19.34, 0.02, 0.18, 2.65}, {12.0, 1.57});
  this->CheckRange(11000, {-7.07, -42.95, -17.99, 0.02, 0.21, 3.13}, {12.0, 1.57});

  // +Y: increase Y
  this->CheckRange(11500, {2.56, -41.84, -16.46, 0.02, 0.23, -2.68}, {12.0, 1.57});
  this->CheckRange(12000, {10.57, -36.46, -14.75, 0.01, 0.24, -2.20}, {12.0, 1.57});
  this->CheckRange(12500, {15.21, -28.04, -12.94, -0.00, 0.26, -1.72}, {12.0, 1.57});
  this->CheckRange(13000, {15.46, -18.47, -10.98, -0.01, 0.27, -1.25}, {12.0, 1.57});

  // -X: decrease X
  this->CheckRange(13500, {11.27, -9.88, -8.96, -0.02, 0.28, -0.77}, {12.0, 1.57});
  this->CheckRange(14000, {3.64, -4.21, -6.82, -0.02, 0.30, -0.29}, {12.0, 1.57});
  this->CheckRange(14500, {-5.72, -2.71, -4.59, -0.03, 0.30, 0.19}, {12.0, 1.57});

  // -Y: decrease Y
  this->CheckRange(15000, {-14.67, -5.69, -2.15, -0.02, 0.22, 0.68}, {12.0, 1.57});
  this->CheckRange(15500, {-21.37, -12.64, -0.51, -0.01, 0.06, 1.15}, {12.0, 1.57});
  this->CheckRange(16000, {-24.23, -22.08, 0.09, 0.01, -0.00, 1.62}, {12.0, 1.57});

  // +X: increase X
  this->CheckRange(16500, {-22.48, -31.83, 0.20, 0.02, -0.05, 2.09}, {12.0, 1.57});
  this->CheckRange(17000, {-16.47, -39.71, -0.05, 0.02, -0.09, 2.57}, {12.0, 1.57});
  this->CheckRange(17500, {-7.55, -43.97, -0.61, 0.01, -0.12, 3.04}, {12.0, 1.57});

  // +Y: increase Y
  this->CheckRange(18000, {2.29, -43.69, -1.45, -0.01, -0.15, -2.77}, {12.0, 1.57});
  this->CheckRange(18500, {10.87, -38.98, -2.53, -0.02, -0.18, -2.29}, {12.0, 1.57});
  this->CheckRange(19000, {16.33, -30.90, -3.83, -0.03, -0.20, -1.82}, {12.0, 1.57});
  this->CheckRange(19500, {17.48, -21.26, -5.34, -0.02, -0.22, -1.34}, {12.0, 1.57});

  // -X: decrease X
  this->CheckRange(20000, {14.10, -12.22, -7.00, -0.01, -0.24, -0.86}, {12.0, 1.57});
  this->CheckRange(20500, {6.97, -5.76, -8.81, 0.00, -0.25, -0.39}, {12.0, 1.57});
  this->CheckRange(21000, {-2.29, -3.31, -10.74, 0.02, -0.27, 0.09}, {12.0, 1.57});

  // -Y: decrease Y
  this->CheckRange(21500, {-11.60, -5.40, -12.79, 0.03, -0.28, 0.57}, {12.0, 1.57});
  this->CheckRange(22000, {-18.89, -11.54, -14.88, 0.04, -0.29, 1.05}, {12.0, 1.57});
  this->CheckRange(22500, {-22.50, -20.31, -17.10, 0.04, -0.31, 1.53}, {12.0, 1.57});

  // +X: increase X
  this->CheckRange(23000, {-21.65, -29.69, -19.53, 0.03, -0.24, 2.02}, {12.0, 1.57});
  this->CheckRange(23500, {-16.42, -37.76, -21.33, 0.01, -0.07, 2.49}, {12.0, 1.57});
  this->CheckRange(24000, {-7.88, -42.67, -21.96, -0.02, -0.00, 2.96}, {12.0, 1.57});
  this->CheckRange(24500, {2.01, -43.16, -22.10, -0.04, 0.05, -2.85}, {12.0, 1.57});

  // +Y: increase Y
  this->CheckRange(25000, {11.04, -39.07, -21.87, -0.03, 0.09, -2.37}, {12.0, 1.57});
  this->CheckRange(25500, {17.19, -31.34, -21.31, -0.01, 0.13, -1.90}, {12.0, 1.57});
  this->CheckRange(26000, {19.12, -21.68, -20.48, 0.01, 0.15, -1.42}, {12.0, 1.57});

  // -X: decrease X
  this->CheckRange(26500, {16.43, -12.26, -19.39, 0.03, 0.18, -0.95}, {12.0, 1.57});
  this->CheckRange(27000, {9.76, -5.16, -18.07, 0.04, 0.20, -0.47}, {12.0, 1.57});
  this->CheckRange(27500, {0.62, -1.91, -16.59, 0.03, 0.22, 0.01}, {12.0, 1.57});

  // -Y: decrease Y
  this->CheckRange(28000, {-8.96, -3.21, -14.93, 0.02, 0.23, 0.49}, {12.0, 1.57});
}

