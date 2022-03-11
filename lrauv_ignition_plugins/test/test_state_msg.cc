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

// Checks that don't change throughout the test
void commonChecks(const lrauv_ignition_plugins::msgs::LRAUVState &_msg)
{
  // Check actuators that don't change throughout the test
  EXPECT_NEAR(0.0, _msg.elevatorangle_(), 1e-4);
  EXPECT_NEAR(0.0, _msg.massposition_(), 1e-6);

  // rph is equal to posRPH
  EXPECT_DOUBLE_EQ(_msg.posrph_().x(), _msg.rph_().x());
  EXPECT_DOUBLE_EQ(_msg.posrph_().y(), _msg.rph_().y());
  EXPECT_DOUBLE_EQ(_msg.posrph_().z(), _msg.rph_().z());
}

//////////////////////////////////////////////////
TEST_F(LrauvTestFixture, State)
{
  // Initial state
  this->fixture->Server()->Run(true, 100, false);
  int maxSleep{100};
  int sleep{0};
  for (; sleep < maxSleep && this->stateMsgs.size() < 100; ++sleep)
  {
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_EQ(100, this->stateMsgs.size());

  auto latest = this->stateMsgs.back();
  commonChecks(latest);

  // Actuators
  EXPECT_NEAR(0.0, latest.propomega_(), 1e-6);
  EXPECT_NEAR(0.0, latest.rudderangle_(), 1e-6);
  EXPECT_NEAR(0.0005, latest.buoyancyposition_(), 1e-6);

  // Position
  ignition::math::Angle initialLat(IGN_DTOR(35.5999984741211));
  ignition::math::Angle initialLon(IGN_DTOR(-121.779998779297));
  EXPECT_NEAR(0.0, latest.depth_(), 1e-6);
  EXPECT_NEAR(initialLat.Degree(), latest.latitudedeg_(), 1e-6);
  EXPECT_NEAR(initialLon.Degree(), latest.longitudedeg_(), 1e-6);

  // Starts aligned with NED world frame (zero pose facing North)
  EXPECT_NEAR(0.0, latest.pos_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.pos_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.pos_().z(), 1e-6);

  EXPECT_NEAR(0.0, latest.posrph_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.posrph_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.posrph_().z(), 1e-6);

  // Velocity
  EXPECT_NEAR(0.0, latest.speed_(), 1e-6);

  // NED world frame
  EXPECT_NEAR(0.0, latest.posdot_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.posdot_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.posdot_().z(), 1e-6);

  // FSK vehicle frame
  EXPECT_NEAR(0.0, latest.rateuvw_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.rateuvw_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.rateuvw_().z(), 1e-6);

  EXPECT_NEAR(0.0, latest.ratepqr_().x(), 1e-6);
  EXPECT_NEAR(0.0, latest.ratepqr_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.ratepqr_().z(), 1e-6);

  // TODO(chapulina) Check sensor data once interpolation is complete
  // https://github.com/osrf/lrauv/issues/5

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

  // The vehicle starts facing North and is propelled forward.
  // We expect to see the position and velocity increase on:
  // ENU: +Y
  // NED: +X
  // FSK: +X
  // Longitude: No change
  // Latitude: increase
  latest = this->stateMsgs.back();
  commonChecks(latest);

  // Actuators
  EXPECT_NEAR(10.0 * IGN_PI, latest.propomega_(), 1e-6);
  EXPECT_NEAR(0.0, latest.rudderangle_(), 1e-3);
  EXPECT_NEAR(0.0005, latest.buoyancyposition_(), 1e-6);

  // Position
  EXPECT_NEAR(0.0, latest.depth_(), 1e-6);
  EXPECT_LT(initialLat.Degree(), latest.latitudedeg_());
  EXPECT_NEAR(initialLon.Degree(), latest.longitudedeg_(), 1e-6);

  // NED world frame: vehicle is going North with no rotation
  EXPECT_LT(28.0, latest.pos_().x());
  EXPECT_NEAR(0.0, latest.pos_().y(), 1e-3);
  EXPECT_NEAR(0.0, latest.pos_().z(), 1e-6);
  EXPECT_NEAR(0.0, latest.posrph_().x(), 1e-3);
  EXPECT_NEAR(0.0, latest.posrph_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.posrph_().z(), 1e-5);

  // Velocity
  EXPECT_NEAR(1.0, latest.speed_(), 0.02);

  // NED world frame: vehicle is going North
  EXPECT_NEAR(1.0, latest.posdot_().x(), 0.02);
  EXPECT_NEAR(0.0, latest.posdot_().y(), 1e-4);
  EXPECT_NEAR(0.0, latest.posdot_().z(), 1e-6);

  // FSK vehicle frame: vehicle is going forward
  EXPECT_NEAR(1.0, latest.rateuvw_().x(), 0.02);
  EXPECT_NEAR(0.0, latest.rateuvw_().y(), 1e-4);
  EXPECT_NEAR(0.0, latest.rateuvw_().z(), 1e-6);

  EXPECT_NEAR(0.0, latest.ratepqr_().x(), 1e-3);
  EXPECT_NEAR(0.0, latest.ratepqr_().y(), 1e-6);
  EXPECT_NEAR(0.0, latest.ratepqr_().z(), 1e-5);

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

  // We expect the vehicle to rotate towards East.
  // The vehicle starts facing North and is propelled forward.
  // We expect to see the position and velocity increase on:
  // ENU: +X, +Y and -yaw
  // NED: +X, +Y and +yaw
  // FSK: +X, +Y and +yaw
  // Longitude: increase
  // Latitude: increase
  latest = this->stateMsgs.back();
  commonChecks(latest);

  // Actuators
  EXPECT_NEAR(10.0 * IGN_PI, latest.propomega_(), 1e-3);
  EXPECT_NEAR(-0.5, latest.rudderangle_(), 0.06);
  EXPECT_NEAR(0.0005, latest.buoyancyposition_(), 1e-6);

  // Position
  EXPECT_NEAR(0.0, latest.depth_(), 1e-3);
  EXPECT_LT(initialLat.Degree(), latest.latitudedeg_());
  EXPECT_LT(initialLon.Degree(), latest.longitudedeg_());

  // NED world frame: vehicle is going North East with positive yaw
  EXPECT_LT(30.0, latest.pos_().x());
  EXPECT_LT(0.4, latest.pos_().y());
  EXPECT_NEAR(0.0, latest.pos_().z(), 1e-3);
  EXPECT_NEAR(0.0, latest.posrph_().x(), 1e-3);
  EXPECT_NEAR(0.0, latest.posrph_().y(), 1e-3);
  EXPECT_LT(0.5, latest.posrph_().z());

  // Velocity
  EXPECT_NEAR(1.0, latest.speed_(), 0.15);

  // NED world frame: vehicle is going North East
  EXPECT_LT(0.6, latest.posdot_().x());
  EXPECT_LT(0.3, latest.posdot_().y());
  EXPECT_NEAR(0.0, latest.posdot_().z(), 1e-3);

  // FSK vehicle frame: vehicle is going mostly forward and rotating right.
  EXPECT_LT(0.8, latest.rateuvw_().x());
  EXPECT_GT(0.2, abs(latest.rateuvw_().y()));
  EXPECT_NEAR(0.0, latest.rateuvw_().z(), 1e-3);

  EXPECT_NEAR(0.0, latest.ratepqr_().x(), 1e-2);
  EXPECT_NEAR(0.0, latest.ratepqr_().y(), 1e-3);
  EXPECT_NEAR(0.1, latest.ratepqr_().z(), 1e-2);

  // Stop propelling and rotating vehicle
  cmdMsg.set_propomegaaction_(0);
  cmdMsg.set_rudderangleaction_(0);

  // Volume below neutral for vehicle to sink
  cmdMsg.set_buoyancyaction_(0.0002);
  cmdMsg.set_dropweightstate_(true);

  // Run server until we collect more states
  this->stateMsgs.clear();
  this->PublishCommandWhile(cmdMsg, [&]()
  {
    return this->stateMsgs.size() < 1000;
  });

  // We expect the vehicle to sink
  latest = this->stateMsgs.back();
  commonChecks(latest);

  // Actuators
  EXPECT_NEAR(0.0, latest.propomega_(), 1e-3);
  EXPECT_NEAR(0.0, latest.rudderangle_(), 1e-3);
  EXPECT_NEAR(0.0002, latest.buoyancyposition_(), 1e-3);

  // Position
  EXPECT_LT(0.2, latest.depth_());

  // NED world frame: higher Z is deeper
  EXPECT_LT(0.2, latest.pos_().z());

  // Velocity
  // NED world frame: sinking to higher Z
  EXPECT_LT(0.02, latest.posdot_().z());

  // FSK vehicle frame: downwards velocity
  EXPECT_LT(0.01, latest.rateuvw_().z());
}

