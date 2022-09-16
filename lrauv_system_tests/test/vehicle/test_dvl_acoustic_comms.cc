/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <lrauv_gazebo_plugins/dvl_velocity_tracking.pb.h>
#include <lrauv_gazebo_plugins/dvl_tracking_target.pb.h>

#include "lrauv_system_tests/TestFixture.hh"
#include "lrauv_system_tests/Util.hh"

#include "TestConstants.hh"

using namespace lrauv_system_tests;
using namespace std::literals::chrono_literals;

using DVLBeamState = lrauv_gazebo_plugins::msgs::DVLBeamState;
using DVLTrackingTarget = lrauv_gazebo_plugins::msgs::DVLTrackingTarget;
using DVLVelocityTracking = lrauv_gazebo_plugins::msgs::DVLVelocityTracking;

static constexpr double beamInclination{GZ_PI / 6.};
static constexpr gz::math::Vector3d sensorPositionInSFMFrame{0., 0.6, -0.16};

// Account for slight roll and limited resolution
static constexpr double kRangeTolerance{0.2};

//////////////////////////////////////////////////
TEST(DVLTest, BottomTrackingAcousticComms)
{
  TestFixture fixture("flat_seabed_two_vehicles.sdf");

  gz::transport::Node node;

  // Command publishers for both vehicles.
  auto cmdPubTethys =
    node.Advertise<gz::msgs::Double>(
      gz::transport::TopicUtils::AsValidTopic(
      "/model/tethys/joint/propeller_joint/cmd_vel"));
  auto cmdPubDaphne =
    node.Advertise<gz::msgs::Double>(
      gz::transport::TopicUtils::AsValidTopic(
      "/model/daphne/joint/propeller_joint/cmd_vel"));

  // DVL sensor callback setup for tethys
  std::function<void(const DVLVelocityTracking &)> dvlCbTethys =
    [&](const DVLVelocityTracking &_msg)
    {
      std::cout << "DBG " << _msg.has_velocity() << std::endl;
      auto linearVelocity = gz::msgs::Convert(_msg.velocity().mean());
      std::cout << linearVelocity.X() << std::endl;
      std::cout << linearVelocity.Y() << std::endl;
      std::cout << linearVelocity.Z() << std::endl;
    };
  node.Subscribe("/tethys/dvl/velocity", dvlCbTethys);

  // Send move command to tethys and run the simulation.
  for (int _ = 0; _ < 20; _++)
  {
    gz::msgs::Double cmdMsg;
    cmdMsg.set_data(2);
    cmdPubTethys.Publish(cmdMsg);
    fixture.Step(10s);
  }
}
