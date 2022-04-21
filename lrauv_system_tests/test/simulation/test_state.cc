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

#include <gtest/gtest.h>

#include <chrono>

#include <ignition/math/Angle.hh>

#include <lrauv_ignition_plugins/lrauv_command.pb.h>
#include <lrauv_ignition_plugins/lrauv_state.pb.h>

#include "lrauv_system_tests/TestFixture.hh"

using namespace std::literals::chrono_literals;

class VehicleStateTest : public ::testing::Test
{
  public: VehicleStateTest() : fixture("buoyant_tethys.sdf", "tethys")
  {
  }

  protected: void SetUp() override
  {
    this->fixture.Step();  // once for pub/subs to connect
  }

  protected: lrauv_system_tests::VehicleStateTestFixture fixture;

  protected: const ignition::math::Angle initialLat{IGN_DTOR(35.5999984741211)};

  protected: const ignition::math::Angle initialLon{IGN_DTOR(-121.779998779297)};
};

// Checks that don't change throughout the test
void CheckInvariants(const lrauv_ignition_plugins::msgs::LRAUVState &_msg)
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
TEST_F(VehicleStateTest, InitialState)
{
  this->fixture.Step(100u);

  // Expect as many state messages as simulation steps
  auto &subscription = this->fixture.StateSubscription();
  EXPECT_TRUE(subscription.WaitForMessages(fixture.Iterations(), 10s));
  const auto latestState = subscription.ReadLastMessage();

  CheckInvariants(latestState);

  // Actuators
  EXPECT_NEAR(0.0, latestState.propomega_(), 1e-6);
  EXPECT_NEAR(0.0, latestState.rudderangle_(), 1e-6);
  EXPECT_NEAR(0.0005, latestState.buoyancyposition_(), 1e-6);

  // Position
  EXPECT_NEAR(0.5, latestState.depth_(), 1e-6);
  EXPECT_NEAR(initialLat.Degree(), latestState.latitudedeg_(), 1e-6);
  EXPECT_NEAR(initialLon.Degree(), latestState.longitudedeg_(), 1e-6);

  // Starts aligned with NED world frame (zero pose facing North)
  EXPECT_NEAR(0.0, latestState.pos_().x(), 1e-6);
  EXPECT_NEAR(0.0, latestState.pos_().y(), 1e-6);
  EXPECT_NEAR(0.5, latestState.pos_().z(), 1e-6);

  EXPECT_NEAR(0.0, latestState.posrph_().x(), 1e-6);
  EXPECT_NEAR(0.0, latestState.posrph_().y(), 1e-6);
  EXPECT_NEAR(0.0, latestState.posrph_().z(), 1e-6);

  // Velocity
  EXPECT_NEAR(0.0, latestState.speed_(), 1e-6);

  // NED world frame
  EXPECT_NEAR(0.0, latestState.posdot_().x(), 1e-6);
  EXPECT_NEAR(0.0, latestState.posdot_().y(), 1e-6);
  EXPECT_NEAR(0.0, latestState.posdot_().z(), 1e-6);

  // FSK vehicle frame
  EXPECT_NEAR(0.0, latestState.rateuvw_().x(), 1e-6);
  EXPECT_NEAR(0.0, latestState.rateuvw_().y(), 1e-6);
  EXPECT_NEAR(0.0, latestState.rateuvw_().z(), 1e-6);

  EXPECT_NEAR(0.0, latestState.ratepqr_().x(), 1e-6);
  EXPECT_NEAR(0.0, latestState.ratepqr_().y(), 1e-6);
  EXPECT_NEAR(0.0, latestState.ratepqr_().z(), 1e-6);

  // TODO(chapulina) Check sensor data once interpolation is complete
  // https://github.com/osrf/lrauv/issues/5
}

//////////////////////////////////////////////////
TEST_F(VehicleStateTest, ThrustState)
{
  lrauv_ignition_plugins::msgs::LRAUVCommand command;

  // Propel vehicle forward by giving the propeller a positive
  // angular velocity. Vehicle is supposed to move at around
  // 1 m/s with 300 RPM. 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  command.set_propomegaaction_(10 * IGN_PI);

  // Neutral buoyancy
  command.set_buoyancyaction_(0.0005);
  command.set_dropweightstate_(true);

  // Step simulation while publishing commands
  auto &publisher = this->fixture.CommandPublisher();
  for (size_t i = 0; i < 5; ++i)
  {
    publisher.Publish(command);
    this->fixture.Step(400u);
  }

  // Expect as many state messages as simulation steps
  auto &subscription = this->fixture.StateSubscription();
  EXPECT_TRUE(subscription.WaitForMessages(this->fixture.Iterations(), 5s));
  const auto latestState = subscription.ReadLastMessage();

  // The vehicle starts facing North and is propelled forward.
  // We expect to see the position and velocity increase on:
  // ENU: +Y
  // NED: +X
  // FSK: +X
  // Longitude: No change
  // Latitude: increase
  CheckInvariants(latestState);

  // Actuators
  EXPECT_NEAR(10.0 * IGN_PI, latestState.propomega_(), 1e-6);
  EXPECT_NEAR(0.0, latestState.rudderangle_(), 1e-3);
  EXPECT_NEAR(0.0005, latestState.buoyancyposition_(), 1e-6);

  // Position
  EXPECT_NEAR(0.5, latestState.depth_(), 1e-6);
  EXPECT_LT(initialLat.Degree(), latestState.latitudedeg_());
  EXPECT_NEAR(initialLon.Degree(), latestState.longitudedeg_(), 1e-6);

  // NED world frame: vehicle is going North with no rotation
  EXPECT_LT(25.0, latestState.pos_().x());
  EXPECT_NEAR(0.0, latestState.pos_().y(), 1e-3);
  EXPECT_NEAR(0.5, latestState.pos_().z(), 1e-6);
  EXPECT_NEAR(0.0, latestState.posrph_().x(), 1e-3);
  EXPECT_NEAR(0.0, latestState.posrph_().y(), 1e-6);
  EXPECT_NEAR(0.0, latestState.posrph_().z(), 1e-5);

  // Velocity
  EXPECT_NEAR(1.0, latestState.speed_(), 0.02);

  // NED world frame: vehicle is going North
  EXPECT_NEAR(1.0, latestState.posdot_().x(), 0.02);
  EXPECT_NEAR(0.0, latestState.posdot_().y(), 1e-4);
  EXPECT_NEAR(0.0, latestState.posdot_().z(), 1e-6);

  // FSK vehicle frame: vehicle is going forward
  EXPECT_NEAR(1.0, latestState.rateuvw_().x(), 0.02);
  EXPECT_NEAR(0.0, latestState.rateuvw_().y(), 1e-4);
  EXPECT_NEAR(0.0, latestState.rateuvw_().z(), 1e-6);

  EXPECT_NEAR(0.0, latestState.ratepqr_().x(), 1e-3);
  EXPECT_NEAR(0.0, latestState.ratepqr_().y(), 1e-6);
  EXPECT_NEAR(0.0, latestState.ratepqr_().z(), 1e-5);
}

//////////////////////////////////////////////////
TEST_F(VehicleStateTest, ThrustAndTurnState)
{
  lrauv_ignition_plugins::msgs::LRAUVCommand command;

  // Propel vehicle forward by giving the propeller a positive
  // angular velocity. Vehicle is supposed to move at around
  // 1 m/s with 300 RPM. 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  command.set_propomegaaction_(10 * IGN_PI);

  // Neutral buoyancy
  command.set_buoyancyaction_(0.0005);
  command.set_dropweightstate_(true);

  // Step simulation while publishing commands
  auto &publisher = this->fixture.CommandPublisher();
  for (size_t i = 0; i < 5; ++i)
  {
    publisher.Publish(command);
    this->fixture.Step(500u);
  }

  // Rotate rudder counter-clockwise when looking from the top,
  // which causes the vehicle to move in a clockwise arc
  command.set_rudderangleaction_(-0.5);

  for (size_t i = 0; i < 2; ++i)
  {
    publisher.Publish(command);
    this->fixture.Step(200u);
  }

  // Expect as many state messages as simulation steps
  auto &subscription = this->fixture.StateSubscription();
  EXPECT_TRUE(subscription.WaitForMessages(this->fixture.Iterations(), 5s));
  const auto latestState = subscription.ReadLastMessage();

  // We expect the vehicle to rotate towards East.
  // The vehicle starts facing North and is propelled forward.
  // We expect to see the position and velocity increase on:
  // ENU: +X, +Y and -yaw
  // NED: +X, +Y and +yaw
  // FSK: +X, +Y and +yaw
  // Longitude: increase
  // Latitude: increase
  CheckInvariants(latestState);

  // Actuators
  EXPECT_NEAR(10.0 * IGN_PI, latestState.propomega_(), 1e-3);
  EXPECT_NEAR(-0.5, latestState.rudderangle_(), 0.06);
  EXPECT_NEAR(0.0005, latestState.buoyancyposition_(), 1e-6);

  // Position
  EXPECT_NEAR(0.5, latestState.depth_(), 1e-3);
  EXPECT_LT(initialLat.Degree(), latestState.latitudedeg_());
  EXPECT_LT(initialLon.Degree(), latestState.longitudedeg_());

  // NED world frame: vehicle is going North East with positive yaw
  EXPECT_LT(30.0, latestState.pos_().x());
  EXPECT_LT(0.4, latestState.pos_().y());
  EXPECT_NEAR(0.5, latestState.pos_().z(), 1e-3);
  EXPECT_NEAR(0.0, latestState.posrph_().x(), 1e-3);
  EXPECT_NEAR(0.0, latestState.posrph_().y(), 1e-3);
  EXPECT_LT(0.5, latestState.posrph_().z());

  // Velocity
  EXPECT_NEAR(1.0, latestState.speed_(), 0.15);

  // NED world frame: vehicle is going North East
  EXPECT_LT(0.6, latestState.posdot_().x());
  EXPECT_LT(0.3, latestState.posdot_().y());
  EXPECT_NEAR(0.0, latestState.posdot_().z(), 1e-3);

  // FSK vehicle frame: vehicle is going mostly forward and rotating right.
  EXPECT_LT(0.8, latestState.rateuvw_().x());
  EXPECT_GT(0.2, abs(latestState.rateuvw_().y()));
  EXPECT_NEAR(0.0, latestState.rateuvw_().z(), 1e-3);

  EXPECT_NEAR(0.0, latestState.ratepqr_().x(), 1e-2);
  EXPECT_NEAR(0.0, latestState.ratepqr_().y(), 1e-3);
  EXPECT_NEAR(0.1, latestState.ratepqr_().z(), 1e-2);
}

//////////////////////////////////////////////////
TEST_F(VehicleStateTest, SinkState)
{
  lrauv_ignition_plugins::msgs::LRAUVCommand command;
  // Volume below neutral for vehicle to sink
  command.set_buoyancyaction_(0.0002);
  command.set_dropweightstate_(true);

  // Step simulation while publishing commands
  auto &publisher = this->fixture.CommandPublisher();
  for (size_t i = 0; i < 5; ++i)
  {
    publisher.Publish(command);
    this->fixture.Step(400u);
  }

  // Expect as many state messages as simulation steps
  auto &subscription = this->fixture.StateSubscription();
  EXPECT_TRUE(subscription.WaitForMessages(this->fixture.Iterations(), 5s));
  const auto latestState = subscription.ReadLastMessage();

  // We expect the vehicle to sink
  CheckInvariants(latestState);

  // Actuators
  EXPECT_NEAR(0.0, latestState.propomega_(), 1e-3);
  EXPECT_NEAR(0.0, latestState.rudderangle_(), 1e-3);
  EXPECT_NEAR(0.0002, latestState.buoyancyposition_(), 1e-3);

  // Position
  EXPECT_LT(0.5, latestState.depth_());

  // NED world frame: higher Z is deeper
  EXPECT_LT(0.5, latestState.pos_().z());

  // Velocity
  // NED world frame: sinking to higher Z
  EXPECT_LT(0.02, latestState.posdot_().z());

  // FSK vehicle frame: downwards velocity
  EXPECT_LT(0.01, latestState.rateuvw_().z());
}
