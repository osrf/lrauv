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

/* In this test, 2 LRAUVs : Tethys (acoustic address : 1) and */
/* Daphne (acoustic address: 2) are placed next to each other. */
/* Tethys is sent a command to move with some propeller speed. */
/* The speed of Tethys w.r.t seabed is observed using the DVL sensor, */
/* and sent to Daphne using acoustic comms. Daphne copies that speed and */
/* tries to catch up with Tethys. */

//////////////////////////////////////////////////
TEST(DVLTest, BottomTrackingAcousticComms)
{
  VehicleCommandTestFixture fixture("flat_seabed_two_vehicles.sdf", "daphne");

  gz::transport::Node node;

  // Acoustic comms publisher.
  auto commsPub = node.Advertise<gz::msgs::Dataframe>(
      "/broker/msgs");

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
      // Send the current velocity of tethys to daphne using acoustic comms.
      auto yVelocity = gz::msgs::Convert(_msg.velocity().mean()).Y();
      gz::msgs::Dataframe msg;
      msg.set_src_address("1");
      msg.set_dst_address("2");
      double scalingFactor = 150;
      msg.set_data(std::to_string(yVelocity * scalingFactor));
      commsPub.Publish(msg);
    };
  node.Subscribe("/tethys/dvl/velocity", dvlCbTethys);

  // Acoustic comms callback for daphne.
  std::function<void(const gz::msgs::Dataframe &)> acousticCbDaphne =
    [&](const gz::msgs::Dataframe &_msg)
    {
      // The "command" for speed is received via acoustic comms.
      gz::msgs::Double cmdMsg;
      cmdMsg.set_data(std::stod(_msg.data()));
      cmdPubDaphne.Publish(cmdMsg);
    };
  node.Subscribe("/2/rx", acousticCbDaphne);

  // Send move command to tethys and run the simulation.
  for (int _ = 0; _ < 10; _++)
  {
    gz::msgs::Double cmdMsg;
    cmdMsg.set_data(2);
    cmdPubTethys.Publish(cmdMsg);
    fixture.Step(10s);
  }

  auto posInitialDaphne = fixture.VehicleObserver().Poses().front().Y();
  auto posFinalDaphne = fixture.VehicleObserver().Poses().back().Y();
  EXPECT_NEAR(posFinalDaphne - posInitialDaphne, 0.25, 0.05);
}
