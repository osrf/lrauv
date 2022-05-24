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

#include <ignition/msgs/battery_state.pb.h>

#include <chrono>

#include <lrauv_ignition_plugins/lrauv_command.pb.h>

#include "lrauv_system_tests/TestFixture.hh"

using namespace ignition;

//////////////////////////////////////////////////
/// Test if the battery discharges with time with the specified
/// discharge power rate, when starting with low charge.
/// Send recharge start/stop commands and verify the battery behaves
/// accordingly.
TEST(BatteryTest, TestDischargeLowCharge)
{
  using TestFixture = lrauv_system_tests::VehicleCommandTestFixture;
  TestFixture fixture("buoyant_tethys_low_battery.sdf", "tethys");
  uint64_t iterations = fixture.Step(100u);

  transport::Node node;
  lrauv_system_tests::Subscription<msgs::BatteryState> batterySubscription;
  batterySubscription.Subscribe(node, "/model/tethys/battery/linear_battery/state");

  fixture.Step(1000u);

  EXPECT_GT(batterySubscription.MessageHistorySize(), 5);

  /* Make sure the battery has drained */
  int n = batterySubscription.MessageHistorySize() - 1;
  double initialCharge = batterySubscription.GetMessageByIndex(0).charge();
  double initialVoltage = batterySubscription.GetMessageByIndex(0).voltage();

  double finalCharge = batterySubscription.GetMessageByIndex(n).charge();
  double finalVoltage = batterySubscription.GetMessageByIndex(n).voltage();

  EXPECT_NEAR(finalCharge, 0, 1e-3);
  EXPECT_NEAR(finalVoltage, 14.4, 0.1);
  batterySubscription.ResetMessageHistory();
  /* The battery is now fully discharged */

  /* Test battery recharge command */
  /* Start charging and check if charge increases with time */
  const unsigned int timeout{5000};
  bool result;
  msgs::Boolean req;
  msgs::Empty rep;
  EXPECT_TRUE(node.Request("/model/tethys/battery/linear_battery/recharge/start", req, timeout, rep, result));

  fixture.Step(1000u);
  EXPECT_GT(batterySubscription.MessageHistorySize(), 5);
  n = batterySubscription.MessageHistorySize() - 1;
  EXPECT_GT(batterySubscription.GetMessageByIndex(n).charge(), finalCharge);
  batterySubscription.ResetMessageHistory();

  /* Stop charging, charge should decrease with time */
  EXPECT_TRUE(node.Request("/model/tethys/battery/linear_battery/recharge/stop", req, timeout, rep, result));
  fixture.Step(1000u);
  EXPECT_GT(batterySubscription.MessageHistorySize(), 5);
  n = batterySubscription.MessageHistorySize() - 1;
  EXPECT_LT(batterySubscription.GetMessageByIndex(n).charge(),
      batterySubscription.GetMessageByIndex(0).charge());
}
