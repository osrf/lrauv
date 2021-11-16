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
#include "lrauv_state.pb.h"

// Check actuators that don't change throughout the test
void unchangedActuators(const lrauv_ignition_plugins::msgs::LRAUVState &_msg)
{
  EXPECT_NEAR(0.0, _msg.elevatorangle_(), 1e-4);
  EXPECT_NEAR(0.0, _msg.massposition_(), 1e-6);
  EXPECT_NEAR(0.0005, _msg.buoyancyposition_(), 1e-6);
}

//////////////////////////////////////////////////
TEST_F(LrauvTestFixture, Command)
{
  // Initial state
  this->fixture->Server()->Run(true, 100, false);
  EXPECT_EQ(100, this->stateMsgs.size());

  auto latest = this->stateMsgs.back();

  // Actuators
  unchangedActuators(latest);
  EXPECT_NEAR(0.0, latest.propomega_(), 1e-6);
  EXPECT_NEAR(0.0, latest.rudderangle_(), 1e-6);

  // Position
  // TODO(chapulina) Shouldn't start sinking
  EXPECT_NEAR(0.0, latest.depth_(), 0.02);
  EXPECT_NEAR(0.0, latest.rph_().x(), 0.007);
  // TODO(chapulina) Shouldn't be pitching
  EXPECT_NEAR(0.0, latest.rph_().y(), 1e-2);
  EXPECT_NEAR(0.0, latest.rph_().z(), 1e-6);
  // TODO(chapulina) Shouldn't be moving
  EXPECT_NEAR(0.0, latest.speed_(), 1e-2);
  EXPECT_NEAR(0.0, latest.latitudedeg_(), 1e-6);
  EXPECT_NEAR(0.0, latest.longitudedeg_(), 1e-6);
  EXPECT_NEAR(0.0, latest.pos_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.pos_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.pos_().z(), 0.015);
  EXPECT_NEAR(0.0, latest.posrph_().x(), 0.007);
  EXPECT_NEAR(0.0, latest.posrph_().y(), 1e-2);
  EXPECT_NEAR(0.0, latest.posrph_().z(), 1e-6);
  EXPECT_NEAR(0.0, latest.posdot_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.posdot_().y(), 1e-5);
  // TODO(chapulina) Shouldn't start sinking
  EXPECT_NEAR(0.0, latest.posdot_().z(), 0.006);
  EXPECT_NEAR(0.0, latest.rateuvw_().x(), 1e-5);
  EXPECT_NEAR(0.0, latest.rateuvw_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.rateuvw_().z(), 0.06);
  EXPECT_NEAR(0.0, latest.ratepqr_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.ratepqr_().y(), 0.002);
  EXPECT_NEAR(0.0, latest.ratepqr_().z(), 1e-6);
  // TODO(chapulina) Check sensor data once interpolation is complete
  // EXPECT_NEAR(0.0, latest.northcurrent_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.eastcurrent_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.temperature_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.salinity_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.density_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.values_(0), 1e-6);
  // EXPECT_NEAR(0.0, latest.values_(1), 1e-6);

  // Propel vehicle forward by giving the propeller a positive angular velocity
  // Vehicle is supposed to move at around 1 m/s with 300 RPM.
  // 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  lrauv_ignition_plugins::msgs::LRAUVCommand cmdMsg;
  cmdMsg.set_propomegaaction_(10 * IGN_PI);

  // Neutral buoyancy
  cmdMsg.set_buoyancyaction_(0.0005);
  cmdMsg.set_dropweightstate_(true);

  // Run server until we collect more states
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->stateMsgs.size() < 2000;
  });

  // The vehicle starts facing West and is propelled forward.
  // We expect to see the position and velocity increase on:
  // ENU: -X
  // NED: -Y
  // FSK: +X
  latest = this->stateMsgs.back();

  // Actuators
  unchangedActuators(latest);
  // TODO(chapulina)
  EXPECT_NEAR(10.0 * IGN_PI, latest.propomega_(), 1e-3);
  EXPECT_NEAR(0.0, latest.rudderangle_(), 1e-3);


  // TODO(chapulina) Shouldn't start sinking
  EXPECT_NEAR(0.0, latest.depth_(), 0.03);
  EXPECT_NEAR(0.0, latest.rph_().x(), 1e-3);
  // TODO(chapulina) Shouldn't be pitching
  EXPECT_NEAR(0.0, latest.rph_().y(), 1e-2);
  EXPECT_NEAR(0.0, latest.rph_().z(), 1e-5);
  // TODO(chapulina) Shouldn't be moving
  EXPECT_NEAR(1.0, latest.speed_(), 0.16);
  EXPECT_NEAR(0.0, latest.latitudedeg_(), 1e-6);
  EXPECT_NEAR(0.0, latest.longitudedeg_(), 1e-3);
  EXPECT_NEAR(0.0, latest.pos_().x(), 1e-3);
  EXPECT_GT(-30.0, latest.pos_().y());
  EXPECT_NEAR(0.0, latest.pos_().z(), 0.03);
  EXPECT_NEAR(0.0, latest.posrph_().x(), 1e-3);
  EXPECT_NEAR(0.0, latest.posrph_().y(), 1e-2);
  EXPECT_NEAR(0.0, latest.posrph_().z(), 1e-5);
  EXPECT_NEAR(0.0, latest.posdot_().x(), 1e-4);
  EXPECT_NEAR(-1.0, latest.posdot_().y(), 0.16);
  // TODO(chapulina) Shouldn't start sinking
  EXPECT_NEAR(0.0, latest.posdot_().z(), 0.006);
  EXPECT_NEAR(1.0, latest.rateuvw_().x(), 0.16);
  EXPECT_NEAR(0.0, latest.rateuvw_().y(), 1e-4);
  EXPECT_NEAR(0.0, latest.rateuvw_().z(), 1e-4);
  EXPECT_NEAR(0.0, latest.ratepqr_().x(), 0.017);
  EXPECT_NEAR(0.0, latest.ratepqr_().y(), 1e-5);
  EXPECT_NEAR(0.0, latest.ratepqr_().z(), 1e-4);
  // TODO(chapulina) Check sensor data once interpolation is complete
  // EXPECT_NEAR(0.0, latest.northcurrent_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.eastcurrent_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.temperature_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.salinity_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.density_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.values_(0), 1e-6);
  // EXPECT_NEAR(0.0, latest.values_(1), 1e-6);

  // Keep propelling vehicle forward
  cmdMsg.set_propomegaaction_(10 * IGN_PI);

  // Rotate rudder counter-clockwise when looking from the top, which causes the
  // vehicle to move in a clockwise arch
  cmdMsg.set_rudderangleaction_(-0.5);

  // Neutral buoyancy
  cmdMsg.set_buoyancyaction_(0.0005);
  cmdMsg.set_dropweightstate_(true);

  // Run server until we collect more states
  this->stateMsgs.clear();
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->stateMsgs.size() < 300;
  });

  // We expect the vehicle to rotate towards North,
  // The vehicle starts facing West and is propelled forward.
  // We expect to see the position and velocity increase on:
  // ENU: -X, +Y and -yaw
  // NED: -Y, +X and +yaw
  // FSK: +X, +Y and +yaw
  latest = this->stateMsgs.back();
  unchangedActuators(latest);
  // TODO(chapulina)
  // EXPECT_NEAR(10.0 * IGN_PI, latest.propomega_(), 1e-3);
  EXPECT_NEAR(-0.5, latest.rudderangle_(), 0.06);

  // Position
  // TODO(chapulina) Shouldn't start sinking
  EXPECT_NEAR(0.0, latest.depth_(), 0.041);
  EXPECT_NEAR(0.0, latest.rph_().x(), 0.0051);
  // TODO(chapulina) Shouldn't be pitching
  EXPECT_NEAR(0.0, latest.rph_().y(), 1e-2);
  EXPECT_LT(1.2, latest.rph_().z());
  // TODO(chapulina) Shouldn't be moving
  EXPECT_NEAR(1.0, latest.speed_(), 0.16);
  EXPECT_NEAR(0.0, latest.latitudedeg_(), 1e-4);
  EXPECT_NEAR(0.0, latest.longitudedeg_(), 1e-3);
  EXPECT_LT(1.9, latest.pos_().x());
  EXPECT_GT(-30.0, latest.pos_().y());
  EXPECT_NEAR(0.0, latest.pos_().z(), 0.05);
  EXPECT_NEAR(0.0, latest.posrph_().x(), 0.0051);
  EXPECT_NEAR(0.0, latest.posrph_().y(), 1e-2);
  EXPECT_LT(1.2, latest.posrph_().z());
  EXPECT_LT(0.75, latest.posdot_().x());
  // EXPECT_NEAR(-1.0, latest.posdot_().y(), 0.16); // ?
  // TODO(chapulina) Shouldn't start sinking
  EXPECT_NEAR(0.0, latest.posdot_().z(), 0.006);
  EXPECT_LT(0.4, latest.rateuvw_().x());
  EXPECT_GT(1.0, latest.rateuvw_().x());
  EXPECT_LT(0.7, latest.rateuvw_().y());
  EXPECT_NEAR(0.0, latest.rateuvw_().z(), 0.006);
  EXPECT_NEAR(0.0, latest.ratepqr_().x(), 0.017);
  EXPECT_NEAR(0.0, latest.ratepqr_().y(), 0.017);
  EXPECT_LT(0.2, latest.ratepqr_().z());
  // TODO(chapulina) Check sensor data once interpolation is complete
  // EXPECT_NEAR(0.0, latest.northcurrent_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.eastcurrent_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.temperature_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.salinity_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.density_(), 1e-6);
  // EXPECT_NEAR(0.0, latest.values_(0), 1e-6);
  // EXPECT_NEAR(0.0, latest.values_(1), 1e-6);
}

