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

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Physics.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Recreate.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
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

#include "TestConstants.hh"

using namespace std::literals::chrono_literals;

class AhrsTest : public ::testing::Test
{
  protected: void SetUp() override {
    ignition::common::Console::SetVerbosity(4);

    this->fixture = std::make_unique<ignition::gazebo::TestFixture>(
        ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
                                    "worlds", "buoyant_tethys.sdf"));

    this->node.Subscribe(
        "/model/tethys/ahrs/imu", &AhrsTest::OnImuMessage, this);

    this->node.Subscribe(
        "/model/tethys/ahrs/magnetometer",
        &AhrsTest::OnMagnetometerMessage, this);

    this->fixture->OnConfigure(
      [&](const ignition::gazebo::Entity &,
          const std::shared_ptr<const sdf::Element> &,
          ignition::gazebo::EntityComponentManager &_ecm,
          ignition::gazebo::EventManager &)
      {
        ignition::gazebo::World world(
            ignition::gazebo::worldEntity(_ecm));
        ignition::gazebo::Entity tethysEntity =
            world.ModelByName(_ecm, "tethys");
        using ignition::gazebo::components::Static;
        _ecm.CreateComponent(tethysEntity, Static(true));
        auto physicsComponent = _ecm.Component<
          ignition::gazebo::components::Physics>(world.Entity());
        this->maxStepSize = physicsComponent->Data().MaxStepSize();
      });

    this->fixture->OnPreUpdate(
      [&](const ignition::gazebo::UpdateInfo &_info,
          ignition::gazebo::EntityComponentManager &_ecm)
      {
        if (this->poseRequest.has_value()) {
          ignition::gazebo::World world(
              ignition::gazebo::worldEntity(_ecm));
          ignition::gazebo::Model tethys(
              world.ModelByName(_ecm, "tethys"));
          tethys.SetWorldPoseCmd(
              _ecm, this->poseRequest.value());
          this->poseRequest.reset();
        }
      });

    this->fixture->OnPostUpdate(
      [&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &)
      {
        this->simTime = _info.simTime;
      });

    this->fixture->Finalize();

    // Initial step to receive IMU and magnetometer messages
    this->Step(200ms);
  }

 protected: void Step(const std::chrono::steady_clock::duration &_step)
 {
   static constexpr bool blocking = true;
   static constexpr bool paused = true;
   const auto simTimeLimit = this->simTime + _step;
   do {
     const double stepSize =
         std::chrono::duration<double>(simTimeLimit - this->simTime).count();
     const uint64_t iterations = std::ceil(stepSize / this->maxStepSize);
     this->fixture->Server()->Run(blocking, iterations, !paused);
   } while (this->simTime < simTimeLimit);
 }

 protected: void SteerInENU(const ignition::math::Quaterniond &_rotation)
 {
   this->poseRequest = ignition::math::Pose3d(
       ignition::math::Vector3d::Zero, _rotation);
 }

 protected: ignition::math::Vector3d LastAngularVelocityMeasurement() const
 {
   std::lock_guard<std::mutex> lock(lastImuMessageMutex);
   return ignition::msgs::Convert(this->lastImuMessage.angular_velocity());
 }

 protected: ignition::math::Vector3d LastLinearAccelerationMeasurement() const
 {
   std::lock_guard<std::mutex> lock(lastImuMessageMutex);
   return ignition::msgs::Convert(this->lastImuMessage.linear_acceleration());
 }

 protected: ignition::math::Vector3d LastMagneticFieldMeasurement() const
 {
   std::lock_guard<std::mutex> lock(lastMagnetometerMessageMutex);
   return ignition::msgs::Convert(this->lastMagnetometerMessage.field_tesla());
 }

 protected: ignition::math::Quaterniond LastOrientationEstimate() const
 {
   std::lock_guard<std::mutex> lock(lastImuMessageMutex);
   return ignition::msgs::Convert(this->lastImuMessage.orientation());
 }

 private: void OnImuMessage(const ignition::msgs::IMU &_msg)
 {
   std::lock_guard<std::mutex> lock(lastImuMessageMutex);
   this->lastImuMessage = _msg;
 }

 private: void OnMagnetometerMessage(const ignition::msgs::Magnetometer &_msg)
 {
   std::lock_guard<std::mutex> lock(lastMagnetometerMessageMutex);
   this->lastMagnetometerMessage = _msg;
 }

 private: ignition::msgs::IMU lastImuMessage;

 private: mutable std::mutex lastImuMessageMutex;

 private: ignition::msgs::Magnetometer lastMagnetometerMessage;

 private: mutable std::mutex lastMagnetometerMessageMutex;

 private: std::optional<ignition::math::Pose3d> poseRequest;

 private: std::unique_ptr<ignition::gazebo::TestFixture> fixture;

 private: std::chrono::steady_clock::duration simTime{
   std::chrono::steady_clock::duration::zero()};

 private: double maxStepSize;

 private: ignition::transport::Node node;
};

// Acceleration due to gravity
static constexpr double g0{9.8};  // m/s^2
// Earth magnetic field
static constexpr double Be{6.0e-6};  // T
static constexpr double Bn{2.3e-5};  // T
static constexpr double Bu{-4.2e-5};  // T

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
TEST_F(AhrsTest, FrameConventionsAreCorrect)
{
  // Tethys starts out facing north
  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(Bn, Be, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
      AreQuaternionsEqual(
          this->LastOrientationEstimate(),
          ignition::math::Quaterniond::Identity,
          orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to face west
  this->SteerInENU({
      0., 0., IGN_DTOR(90.)});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(-Be, Bn, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
      AreQuaternionsEqual(
          this->LastOrientationEstimate(),
          ignition::math::Quaterniond(
              0., 0., IGN_DTOR(-90.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to face south
  this->SteerInENU({
      0., 0., IGN_DTOR(180.)});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(-Bn, -Be, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
      AreQuaternionsEqual(
          this->LastOrientationEstimate(),
          ignition::math::Quaterniond(
              0., 0., IGN_DTOR(-180.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to face east
  this->SteerInENU({
      0., 0., IGN_DTOR(-90.)});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        -ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(Be, -Bn, -Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
      AreQuaternionsEqual(
          this->LastOrientationEstimate(),
              ignition::math::Quaterniond(
                  0., 0., IGN_DTOR(90.)),
          orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to a vertical position heading upwards
  this->SteerInENU({
      IGN_DTOR(90.), 0., 0.});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        ignition::math::Vector3d::UnitX * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(Bu, Be, Bn),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
      AreQuaternionsEqual(
          this->LastOrientationEstimate(),
          ignition::math::Quaterniond(
              0., IGN_DTOR(90.), 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to a vertical position heading downwards
  this->SteerInENU({
      IGN_DTOR(-90.), -0., 0.});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        -ignition::math::Vector3d::UnitX * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(-Bu, Be, -Bn),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
      AreQuaternionsEqual(
          this->LastOrientationEstimate(),
          ignition::math::Quaterniond(
              0., IGN_DTOR(-90.), 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to roll towards starboard
  this->SteerInENU({
      0., IGN_DTOR(90.), 0.});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        -ignition::math::Vector3d::UnitY * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(Bn, -Bu, -Be),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
      AreQuaternionsEqual(
          this->LastOrientationEstimate(),
          ignition::math::Quaterniond(
              IGN_DTOR(90.), 0., 0.),
          orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to roll towards port
  this->SteerInENU({
      0., IGN_DTOR(-90.), 0.});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        ignition::math::Vector3d::UnitY * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(Bn, Bu, Be),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
    AreQuaternionsEqual(
        this->LastOrientationEstimate(),
        ignition::math::Quaterniond(
            IGN_DTOR(-90.), 0., 0.),
        orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();

  // Steer Tethys to do a half barrel roll
  this->SteerInENU({
      0., IGN_DTOR(180.), 0.});
  this->Step(200ms);

  EXPECT_TRUE(
    this->LastAngularVelocityMeasurement().Equal(
        ignition::math::Vector3d::Zero,
        angularVelocityTolerance))
      << "Last angular velocity measurement was "
      << this->LastAngularVelocityMeasurement();

  EXPECT_TRUE(
    this->LastLinearAccelerationMeasurement().Equal(
        ignition::math::Vector3d::UnitZ * g0,
        linearAccelerationTolerance))
      << "Last linear acceleration measurement was "
      << this->LastLinearAccelerationMeasurement();

  EXPECT_TRUE(
    this->LastMagneticFieldMeasurement().Equal(
        ignition::math::Vector3d(Bn, -Be, Bu),
        magneticFieldTolerance))
      << "Last magnetic field measurement was "
      << this->LastMagneticFieldMeasurement();

  EXPECT_TRUE(
    AreQuaternionsEqual(
        this->LastOrientationEstimate(),
        ignition::math::Quaterniond(
            IGN_DTOR(180.), 0., 0.),
        orientationTolerance))
      << "Last orientation estimate was "
      << this->LastOrientationEstimate();
}
