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
TEST(DVLTest, NoTracking)
{
  VehicleCommandTestFixture fixture("bottomless_pit.sdf", "tethys");

  Subscription<DVLVelocityTracking> velocitySubscription;
  velocitySubscription.Subscribe(fixture.Node(), "/tethys/dvl/velocity", 1);

  fixture.Step(10s);

  ASSERT_TRUE(velocitySubscription.WaitForMessages(10, 10s));

  const DVLVelocityTracking message =
      velocitySubscription.ReadLastMessage();
  EXPECT_FALSE(message.has_target());
  EXPECT_FALSE(message.has_velocity());
  for (int i = 0; i < message.beams_size(); ++i)
  {
    EXPECT_FALSE(message.beams(i).locked())
        << "Beam #" << message.beams(i).id() << " is locked";
  }
}

//////////////////////////////////////////////////
TEST(DVLTest, BottomTracking)
{
  VehicleCommandTestFixture fixture("flat_seabed.sdf", "tethys");
  constexpr double seaBedDepth{20.};
  // Assume zero roll and arbitrary resolution
  const double expectedBeamRange =
      (seaBedDepth + sensorPositionInSFMFrame.Z()) /
      std::cos(beamInclination);

  Subscription<DVLVelocityTracking> velocitySubscription;
  velocitySubscription.Subscribe(fixture.Node(), "/tethys/dvl/velocity", 1);

  // Step a few iterations for simulation to setup itself
  fixture.Step(2s);

  ASSERT_TRUE(velocitySubscription.WaitForMessages(1, 10s));
  {
    const DVLVelocityTracking message =
        velocitySubscription.ReadLastMessage();
    ASSERT_TRUE(message.has_target());
    const DVLTrackingTarget & target = message.target();
    EXPECT_EQ(target.type(), DVLTrackingTarget::DVL_TARGET_BOTTOM);
    EXPECT_NEAR(target.range().mean(), expectedBeamRange, kRangeTolerance);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked()) << "Beam #" << beam.id() << " not locked";
      EXPECT_NEAR(beam.range().mean(), expectedBeamRange, kRangeTolerance)
          << "Beam #" << beam.id() << " range is off";
    }
    ASSERT_TRUE(message.has_velocity());
    const gz::math::Vector3d linearVelocityEstimate =
        gz::msgs::Convert(message.velocity().mean());
    constexpr double kVelocityTolerance{1e-2};  // account for noise
    EXPECT_NEAR(linearVelocityEstimate.X(), 0., kVelocityTolerance);
    EXPECT_NEAR(linearVelocityEstimate.Y(), 0., kVelocityTolerance);
    EXPECT_NEAR(linearVelocityEstimate.Z(), 0., kVelocityTolerance);
  }

  // Propel vehicle forward by giving the propeller a positive
  // angular velocity. Vehicle is supposed to move at around
  // 1 m/s with 300 RPM. 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  lrauv_gazebo_plugins::msgs::LRAUVCommand command;
  command.set_propomegaaction_(10. * GZ_PI);

  // Rotate rudder clockwise when looking from the top,
  // which causes the vehicle to move in a counter-clockwise arch
  command.set_rudderangleaction_(0.8);

  // Neutral buoyancy
  command.set_dropweightstate_(true);
  command.set_buoyancyaction_(0.0005);

  // Step simulation for some time for DVL estimates to estabilize
  auto & publisher = fixture.CommandPublisher();
  for (int _ = 0; _ < 5; ++_)
  {
    publisher.Publish(command);
    fixture.Step(10s);
  }
  ASSERT_TRUE(velocitySubscription.WaitForMessages(50, 10s));

  {
    const DVLVelocityTracking message =
        velocitySubscription.ReadLastMessage();
    ASSERT_TRUE(message.has_target());
    const DVLTrackingTarget & target = message.target();
    EXPECT_EQ(target.type(), DVLTrackingTarget::DVL_TARGET_BOTTOM);
    EXPECT_NEAR(target.range().mean(), expectedBeamRange, kRangeTolerance);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked()) << "Beam #" << beam.id() << " not locked";
      EXPECT_NEAR(beam.range().mean(), expectedBeamRange, kRangeTolerance)
          << "Beam #" << beam.id() << " range is off";
    }
    ASSERT_TRUE(message.has_velocity());
    const gz::math::Vector3d linearVelocityEstimate =
        gz::msgs::Convert(message.velocity().mean());

    const auto &linearVelocities =
        fixture.VehicleObserver().LinearVelocities();
    const auto &angularVelocities =
        fixture.VehicleObserver().AngularVelocities();
    const auto &poses = fixture.VehicleObserver().Poses();

    // Linear velocities w.r.t. to sea bottom are reported
    // in a sensor affixed, SFM frame.
    const gz::math::Vector3d expectedLinearVelocityEstimate =
        poses.back().Rot().RotateVectorReverse(
            linearVelocities.back() + angularVelocities.back().Cross(
                poses.back().Rot().RotateVector(sensorPositionInSFMFrame)));

    constexpr double kVelocityTolerance{1e-2};  // account for noise
    EXPECT_NEAR(expectedLinearVelocityEstimate.X(),
                linearVelocityEstimate.X(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Y(),
                linearVelocityEstimate.Y(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Z(),
                linearVelocityEstimate.Z(), kVelocityTolerance);
  }
}

//////////////////////////////////////////////////
TEST(DVLTest, WaterMassTracking)
{
  VehicleCommandTestFixture fixture("underwater_currents.sdf", "tethys");
  constexpr gz::math::Vector3d waterCurrentVelocity{-1., 0.5, 0.};

  Subscription<DVLVelocityTracking> velocitySubscription;
  velocitySubscription.Subscribe(fixture.Node(), "/tethys/dvl/velocity", 1);

  // Step a few iterations for simulation to setup itself
  fixture.Step(2s);

  ASSERT_TRUE(velocitySubscription.WaitForMessages(1, 10s));
  {
    const DVLVelocityTracking message =
        velocitySubscription.ReadLastMessage();
    ASSERT_TRUE(message.has_target());
    const DVLTrackingTarget & target = message.target();
    EXPECT_EQ(target.type(), DVLTrackingTarget::DVL_TARGET_WATER_MASS);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked())
          << "Beam #" << beam.id() << " not locked";
    }
    ASSERT_TRUE(message.has_velocity());
    const gz::math::Vector3d linearVelocityEstimate =
        gz::msgs::Convert(message.velocity().mean());
    constexpr double kVelocityTolerance{1e-1};  // account for noise
    EXPECT_NEAR(linearVelocityEstimate.X(),
                -waterCurrentVelocity.X(), kVelocityTolerance);
    EXPECT_NEAR(linearVelocityEstimate.Y(),
                -waterCurrentVelocity.Y(), kVelocityTolerance);
    EXPECT_NEAR(linearVelocityEstimate.Z(),
                -waterCurrentVelocity.Z(), kVelocityTolerance);
  }

  // Propel vehicle forward by giving the propeller a positive
  // angular velocity. Vehicle is supposed to move at around
  // 1 m/s with 300 RPM. 300 RPM = 300 * 2 pi / 60 = 10 pi rad/s
  lrauv_ignition_plugins::msgs::LRAUVCommand command;
  command.set_propomegaaction_(10. * GZ_PI);

  // Rotate rudder clockwise when looking from the top,
  // which causes the vehicle to move in a counter-clockwise arch
  command.set_rudderangleaction_(0.8);

  // Neutral buoyancy
  command.set_dropweightstate_(true);
  command.set_buoyancyaction_(0.0005);

  // Step simulation for some time for DVL estimates to estabilize
  auto & publisher = fixture.CommandPublisher();
  for (int _ = 0; _ < 5; ++_)
  {
    publisher.Publish(command);
    fixture.Step(10s);
  }
  ASSERT_TRUE(velocitySubscription.WaitForMessages(50, 10s));

  {
    const DVLVelocityTracking message =
        velocitySubscription.ReadLastMessage();
    ASSERT_TRUE(message.has_target());
    const DVLTrackingTarget & target = message.target();
    EXPECT_EQ(target.type(), DVLTrackingTarget::DVL_TARGET_WATER_MASS);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked())
          << "Beam #" << beam.id() << " not locked";
    }
    ASSERT_TRUE(message.has_velocity());
    const gz::math::Vector3d linearVelocityEstimate =
        gz::msgs::Convert(message.velocity().mean());

    const auto &linearVelocities =
        fixture.VehicleObserver().LinearVelocities();
    const auto &angularVelocities =
        fixture.VehicleObserver().AngularVelocities();
    const auto &poses = fixture.VehicleObserver().Poses();

    // Linear velocities w.r.t. to underwater currents
    // are reported in a sensor affixed, SFM frame.
    const gz::math::Vector3d expectedLinearVelocityEstimate =
        poses.back().Rot().RotateVectorReverse(
            linearVelocities.back() - waterCurrentVelocity +
            angularVelocities.back().Cross(
                poses.back().Rot().RotateVector(sensorPositionInSFMFrame)));
    constexpr double kVelocityTolerance{1e-1};  // account for noise
    EXPECT_NEAR(expectedLinearVelocityEstimate.X(),
                linearVelocityEstimate.X(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Y(),
                linearVelocityEstimate.Y(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Z(),
                linearVelocityEstimate.Z(), kVelocityTolerance);
  }
}
