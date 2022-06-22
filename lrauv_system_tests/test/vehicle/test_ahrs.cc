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

#include <gz/sim/components/Static.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

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
    gz::math::Vector3d angularVelocity;
    gz::math::Vector3d linearAcceleration;
    gz::math::Quaterniond orientation;
    gz::math::Vector3d magneticField;
  };

  public: Measurement ReadLastMeasurement()
  {
    const gz::msgs::IMU imuMessage =
        this->imuSubscription.ReadLastMessage();
    const gz::msgs::Magnetometer magnetometerMessage =
        this->magnetometerSubscription.ReadLastMessage();
    return {
      gz::msgs::Convert(imuMessage.angular_velocity()),
      gz::msgs::Convert(imuMessage.linear_acceleration()),
      gz::msgs::Convert(imuMessage.orientation()),
      gz::msgs::Convert(magnetometerMessage.field_tesla())
    };
  }

  protected: void OnConfigure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    gz::sim::World world(
        gz::sim::worldEntity(_ecm));
    gz::sim::Entity vehicleEntity =
        world.ModelByName(_ecm, this->VehicleName());
    using gz::sim::components::Static;
    _ecm.CreateComponent(vehicleEntity, Static(true));
  }

  private: gz::transport::Node node;

  private: Subscription<gz::msgs::IMU> imuSubscription;

  private: Subscription<gz::msgs::Magnetometer> magnetometerSubscription;
};

// Acceleration due to gravity
static constexpr double g0{9.8};  // m/s^2
// Earth magnetic field
static constexpr double Be{5.5645e-6};    // T
static constexpr double Bn{22.8758e-6};   // T
static constexpr double Bu{-42.3884e-6};  // T

static constexpr double angularVelocityTolerance{3e-2};
static constexpr double linearAccelerationTolerance{3e-2};
static constexpr double magneticFieldTolerance{1e-8};
static constexpr double orientationTolerance{1e-4};

inline bool AreQuaternionsEqual(
    const gz::math::Quaterniond &_qa,
    const gz::math::Quaterniond &_qb,
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
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -gz::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(Bn, Be, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          gz::math::Quaterniond::Identity,
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle face west
  auto &manipulator = fixture.VehicleManipulator();
  manipulator.SetOrientation({0., 0., GZ_DTOR(90.)});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -gz::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(-Be, Bn, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          gz::math::Quaterniond(
              0., 0., GZ_DTOR(-90.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle face south
  manipulator.SetOrientation({0., 0., GZ_DTOR(180.)});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -gz::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(-Bn, -Be, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          gz::math::Quaterniond(
              0., 0., GZ_DTOR(-180.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle face east
  manipulator.SetOrientation({0., 0., GZ_DTOR(-90.)});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -gz::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(Be, -Bn, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          gz::math::Quaterniond(
              0., 0., GZ_DTOR(90.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Put vehicle in a vertical position heading upwards
  manipulator.SetOrientation({GZ_DTOR(90.), 0., 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        gz::math::Vector3d::UnitX * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(Bu, Be, Bn),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          gz::math::Quaterniond(
              0., GZ_DTOR(90.), 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Put vehicle in a vertical position heading downwards
  manipulator.SetOrientation({GZ_DTOR(-90.), -0., 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -gz::math::Vector3d::UnitX * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(-Bu, Be, -Bn),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          gz::math::Quaterniond(
              0., GZ_DTOR(-90.), 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle roll towards starboard
  manipulator.SetOrientation({0., GZ_DTOR(90.), 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        -gz::math::Vector3d::UnitY * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(Bn, -Bu, -Be),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
      AreQuaternionsEqual(
          measurement.orientation,
          gz::math::Quaterniond(
              GZ_DTOR(90.), 0., 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle roll towards port
  manipulator.SetOrientation({0., GZ_DTOR(-90.), 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        gz::math::Vector3d::UnitY * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(Bn, Bu, Be),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
    AreQuaternionsEqual(
        measurement.orientation,
        gz::math::Quaterniond(
            GZ_DTOR(-90.), 0., 0.),
        orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;

  // Make vehicle do a half barrel roll
  manipulator.SetOrientation({0., GZ_DTOR(180.), 0.});

  EXPECT_LT(0, fixture.Step(200ms));
  measurement = fixture.ReadLastMeasurement();

  EXPECT_TRUE(
    measurement.angularVelocity.Equal(
        gz::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << measurement.angularVelocity;

  EXPECT_TRUE(
    measurement.linearAcceleration.Equal(
        gz::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << measurement.linearAcceleration;

  EXPECT_TRUE(
    measurement.magneticField.Equal(
        gz::math::Vector3d(Bn, -Be, Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << measurement.magneticField;

  EXPECT_TRUE(
    AreQuaternionsEqual(
        measurement.orientation,
        gz::math::Quaterniond(
            GZ_DTOR(180.), 0., 0.),
        orientationTolerance))
      << "Last orientation estimate was "
      << measurement.orientation;
}
