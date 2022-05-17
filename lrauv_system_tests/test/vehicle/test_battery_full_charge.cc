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

#include "lrauv_system_tests/TestFixture.hh"

using namespace ignition;

std::vector<msgs::BatteryState> batteryMsgs;

void recordBatteryMsgs(const msgs::BatteryState &_msg)
{
  batteryMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
/// Test if the battery discharges with time with the specified
/// disacharge power rate, when starting with full charge.
TEST(BatteryTest, TestDischargeFullCharged)
{
  using TestFixture = lrauv_system_tests::TestFixtureWithVehicle;
  TestFixture fixture("buoyant_tethys.sdf", "tethys");
  uint64_t iterations = fixture.Step(100u);

  transport::Node node;
  node.Subscribe("/model/tethys/battery/linear_battery/state",
      &recordBatteryMsgs);

  fixture.Step(1000u);

  int n = batteryMsgs.size() - 1;
  double initialCharge = batteryMsgs[0].charge();
  
  /* Plugin started wiht 50% charge */
  EXPECT_NEAR(initialCharge, 400, 0.2);

  double initialVoltage = batteryMsgs[0].voltage();
  double initialTime = batteryMsgs[0].header().stamp().sec() + 
    batteryMsgs[0].header().stamp().nsec()/1000000000.0;

  double finalCharge = batteryMsgs[n].charge();
  double finalVoltage = batteryMsgs[n].voltage();
  double finalTime = batteryMsgs[n].header().stamp().sec() + 
    batteryMsgs[n].header().stamp().nsec()/1000000000.0;

  double dischargePower =  (finalVoltage + initialVoltage) * 0.5 *
    (finalCharge - initialCharge) * 3600 / (finalTime - initialTime);
  EXPECT_NEAR(dischargePower, -28.8, 0.5);

  batteryMsgs.clear();
}
