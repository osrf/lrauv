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

#include <ignition/gazebo/components/Static.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/magnetometer.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>
#include <utility>

#include "lrauv_system_tests/TestFixture.hh"
#include "lrauv_system_tests/Util.hh"

#include "TestConstants.hh"

using namespace lrauv_system_tests;
using namespace std::literals::chrono_literals;

class AHRSTestFixture : public TestFixtureWithVehicle
{
  public: AHRSTestFixture(
      const std::string &_worldName,
      const std::string &_vehicleName)
    : TestFixtureWithVehicle(_worldName, _vehicleName)
  {
    constexpr size_t historyDepth = 1u;
    const std::string imuTopicName =
        "/" + _vehicleName + "/ahrs/imu";
    this->imuSubscription.Subscribe(
        this->node, imuTopicName, historyDepth);
    const std::string magnetometerTopicName =
        "/" + _vehicleName + "/ahrs/magnetometer";
    this->magnetometerSubscription.Subscribe(
        this->node, magnetometerTopicName, historyDepth);
  }

  public: struct Measurement
  {
    ignition::math::Vector3d angularVelocity;
    ignition::math::Vector3d linearAcceleration;
    ignition::math::Quaterniond orientation;
    ignition::math::Vector3d magneticField;
  };

  public: Measurement ReadLastMeasurement()
  {
    const ignition::msgs::IMU imuMessage =
        this->imuSubscription.ReadLastMessage();
    const ignition::msgs::Magnetometer magnetometerMessage =
        this->magnetometerSubscription.ReadLastMessage();
    return {
      ignition::msgs::Convert(imuMessage.angular_velocity()),
      ignition::msgs::Convert(imuMessage.linear_acceleration()),
      ignition::msgs::Convert(imuMessage.orientation()),
      ignition::msgs::Convert(magnetometerMessage.field_tesla())
    };
  }

  protected: void OnConfigure(
      const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &) override
  {
    ignition::gazebo::World world(
        ignition::gazebo::worldEntity(_ecm));
    ignition::gazebo::Entity vehicleEntity =
        world.ModelByName(_ecm, this->VehicleName());
    using ignition::gazebo::components::Static;
    _ecm.CreateComponent(vehicleEntity, Static(true));
  }

  private: ignition::transport::Node node;

  private: Subscription<ignition::msgs::IMU> imuSubscription;

  private: Subscription<ignition::msgs::Magnetometer> magnetometerSubscription;
};

// Acceleration due to gravity
static constexpr double g0{9.8};  // m/s^2
// Earth magnetic field
static constexpr double Be{5.5645e-6};    // T
static constexpr double Bn{22.8758e-6};   // T
static constexpr double Bu{-42.3884e-6};  // T

static constexpr double angularVelocityTolerance{1e-4};
static constexpr double linearAccelerationTolerance{1e-4};
static constexpr double magneticFieldTolerance{1e-8};
static constexpr double orientationTolerance{1e-4};

inline bool AreQuaternionsEqual(
    const ignition::math::Quaterniond &_qa,
    const ignition::math::Quaterniond &_qb,
    const double _tolerance)
{
  // Use geodesic distance between quaternions
  return std::acos(2. * std::pow(_qa.Dot(_qb), 2) - 1.) < _tolerance;
}

//////////////////////////////////////////////////
TEST(AHRSTest, FrameConventionsAreCorrect)
{
  AHRSTestFixture fixture("buoyant_tethys.sdf", "tethys");

  EXPECT_LT(0, fixture.Step(200ms));

  // Tethys starts out facing north
  auto measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(Bn, Be, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          ignition::math::Quaterniond::Identity,
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle face west
  auto &manipulator = fixture.VehicleManipulator();
  manipulator.SetOrientation({0., 0., IGN_DTOR(90.)});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(-Be, Bn, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          ignition::math::Quaterniond(
              0., 0., IGN_DTOR(-90.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle face south
  manipulator.SetOrientation({0., 0., IGN_DTOR(180.)});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(-Bn, -Be, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          ignition::math::Quaterniond(
              0., 0., IGN_DTOR(-180.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle face east
  manipulator.SetOrientation({0., 0., IGN_DTOR(-90.)});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(Be, -Bn, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          ignition::math::Quaterniond(
              0., 0., IGN_DTOR(90.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Put vehicle in a vertical position heading upwards
  manipulator.SetOrientation({IGN_DTOR(90.), 0., 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        ignition::math::Vector3d::UnitX * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(Bu, Be, Bn),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          ignition::math::Quaterniond(
              0., IGN_DTOR(90.), 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Put vehicle in a vertical position heading downwards
  manipulator.SetOrientation({IGN_DTOR(-90.), -0., 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -ignition::math::Vector3d::UnitX * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(-Bu, Be, -Bn),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          ignition::math::Quaterniond(
              0., IGN_DTOR(-90.), 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle roll towards starboard
  manipulator.SetOrientation({0., IGN_DTOR(90.), 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -ignition::math::Vector3d::UnitY * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(Bn, -Bu, -Be),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          ignition::math::Quaterniond(
              IGN_DTOR(90.), 0., 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle roll towards port
  manipulator.SetOrientation({0., IGN_DTOR(-90.), 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        ignition::math::Vector3d::UnitY * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(Bn, Bu, Be),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
    AreQuaternionsEqual(
        measurement.orientation,
        ignition::math::Quaterniond(
            IGN_DTOR(-90.), 0., 0.),
        orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle do a half barrel roll
  manipulator.SetOrientation({0., IGN_DTOR(180.), 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        ignition::math::Vector3d(Bn, -Be, Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
    AreQuaternionsEqual(
        measurement.orientation,
        ignition::math::Quaterniond(
            IGN_DTOR(180.), 0., 0.),
        orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;
}
