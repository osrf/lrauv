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

#include "helper/LrauvTestFixture.hh"
#include "lrauv_command.pb.h"

#include <fstream>

//////////////////////////////////////////////////
TEST_F(LrauvTestFixtureAtDepth, DropWeightRelease)
{
  // Max iterations
  const int maxIterations{3800};

  // Surface is at 0m
  const double targetZ{0};
  // Tolerance, the vehicle will breach the surface
  const double tol{0.5};
  // Starting point of the vehicle
  const double startZ{-10};

  // Get initial Z
  this->fixture->Server()->Run(true, 100, false);
  EXPECT_EQ(100, this->iterations);
  EXPECT_EQ(100, this->tethysPoses.size());
  EXPECT_NEAR(startZ, this->tethysPoses.back().Pos().Z(), 0.05);

  // Tell the vehicle to release the weight
  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_dropweightstate_(false);

  // Neutral buoyancy
  cmdMsg.set_buoyancyaction_(0.0005);

  // Run the server for fixed iterations
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->iterations <= maxIterations;
  });

  for (auto pose : this->tethysPoses)
  {
    // Make sure that the vehicle just breaches the surface of the the water
    EXPECT_GT(targetZ + tol, pose.Pos().Z());

    // MAke sure vehicle goes up.
    EXPECT_LT(startZ - tol, pose.Pos().Z());
  }

  // Actually ran the server.
  EXPECT_LT(100, this->iterations);
  EXPECT_LT(100, this->tethysPoses.size());

  // Make sure we do get near the surface
  EXPECT_NEAR(targetZ, this->tethysPoses.back().Pos().Z(), tol);
}

