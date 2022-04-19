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

#ifndef LRAUV_SYSTEM_TESTS__TEST_FIXTURE_HH
#define LRAUV_SYSTEM_TESTS__TEST_FIXTURE_HH

#include <chrono>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/gazebo/components/Physics.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include <lrauv_ignition_plugins/lrauv_command.pb.h>
#include <lrauv_ignition_plugins/lrauv_state.pb.h>

#include "lrauv_system_tests/ModelManipulator.hh"
#include "lrauv_system_tests/ModelObserver.hh"
#include "lrauv_system_tests/Subscription.hh"

#include "TestConstants.hh"

namespace lrauv_system_tests
{

/// An augmented Ignition Gazebo TestFixture class
/// to ease unit an integration testing of the LRAUV
/// simulation.
class TestFixture
{
  /// Constructor.
  /// \param[in] _worldName Base name of the world SDF,
  /// to be found among this package's ``worlds``.
  public: TestFixture(const std::string &_worldName)
    : fixture(ignition::common::joinPaths(
          PROJECT_SOURCE_PATH, "worlds", _worldName))
  {
  }

  virtual ~TestFixture() = default;

  /// Pause the simulation in this fixture.
  public: void Pause() { this->paused = true; }

  /// Returns the underlying Ignition Gazebo server.
  public: ignition::gazebo::Server *Simulator()
  {
    if (!this->initialized)
    {
      this->fixture
          .OnConfigure(
              [&](const ignition::gazebo::Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  ignition::gazebo::EntityComponentManager &_ecm,
                  ignition::gazebo::EventManager &_eventManager)
              {
                const ignition::gazebo::Entity worldEntity =
                    ignition::gazebo::worldEntity(_ecm);
                ignition::gazebo::World world(worldEntity);
                auto physicsComponent = _ecm.Component<
                  ignition::gazebo::components::Physics>(world.Entity());
                this->maxStepSize = physicsComponent->Data().MaxStepSize();
                this->OnConfigure(_entity, _sdf, _ecm, _eventManager);
              })
          .OnPreUpdate(
              [&](const ignition::gazebo::UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm)
              {
                this->OnPreUpdate(_info, _ecm);
              })
          .OnPostUpdate(
              [&](const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm)
              {
                this->OnPostUpdate(_info, _ecm);
                this->info = _info;
              })
          .Finalize();
      this->initialized = true;
    }
    return this->fixture.Server().get();
  }

  /// Advance the simulation one step forward.
  /// \return number of simulation steps taken.
  public: uint64_t Step()
  {
    this->Simulator()->RunOnce(this->paused);
    return 1u;
  }

  /// Advance the simulation.
  /// \param[in] _steps Number of simulation steps to take.
  /// \return number of simulation steps actually taken.
  public: uint64_t Step(uint64_t _steps)
  {
    static constexpr bool blocking = true;
    uint64_t initial_iterations = this->Iterations();
    this->Simulator()->Run(blocking, _steps, this->paused);
    return this->Iterations() - initial_iterations;
  }

  /// Advance the simulation.
  /// \param[in] _step Step in simulation time to take.
  /// \return number of simulation steps taken.
  public: uint64_t Step(const std::chrono::steady_clock::duration &_step)
  {
    static constexpr bool blocking = true;
    uint64_t iterations = 0u;
    // Fetch simulator early to ensure it is initialized
    auto simulator = this->Simulator();
    const auto deadline = this->info.simTime + _step;
    do {
      const double stepSize =
          std::chrono::duration<double>(deadline - this->info.simTime).count();
      uint64_t previous_iterations = this->Iterations();
      simulator->Run(
          blocking, std::ceil(stepSize / this->maxStepSize), this->paused);
      iterations += this->Iterations() - previous_iterations;
    } while (this->info.simTime < deadline);
    return iterations;
  }

  /// Returns the total number of simulation iterations so far.
  public: uint64_t Iterations() const { return this->info.iterations; }

  /// To be optionally overriden by subclasses.
  /// \see ignition::gazebo::TestFixture
  protected: virtual void OnConfigure(
      const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &)
  {
  }

  /// To be optionally overriden by subclasses.
  /// \see ignition::gazebo::TestFixture
  protected: virtual void OnPreUpdate(
    const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &)
  {
  }

  /// To be optionally overriden by subclasses.
  /// \see ignition::gazebo::TestFixture
  protected: virtual void OnPostUpdate(
    const ignition::gazebo::UpdateInfo &,
    const ignition::gazebo::EntityComponentManager &)
  {
  }

  private: bool initialized{false};

  private: bool paused{false};

  private: double maxStepSize;

  private: ignition::gazebo::UpdateInfo info;

  private: ignition::gazebo::TestFixture fixture;
};

/// A test fixture with a single vehicle.
class TestFixtureWithVehicle : public TestFixture
{
  /// Constructor.
  /// \param[in] _worldName Base name of the world SDF,
  /// to be found among this package's ``worlds``.
  /// \param[in] _vehicleName Name of the vehicle model.
  public: TestFixtureWithVehicle(
      const std::string &_worldName,
      const std::string &_vehicleName)
    : TestFixture(_worldName),
      vehicleManipulator(_vehicleName),
      vehicleObserver(_vehicleName),
      vehicleName(_vehicleName)
  {
  }

  /// Returns the name of the vehicle.
  public: const std::string &VehicleName() const
  {
    return this->vehicleName;
  }

  /// Returns a manipulator for the vehicle.
  public: ModelManipulator &VehicleManipulator()
  {
    return this->vehicleManipulator;
  }

  /// Returns an observer for the vehicle.
  public: const ModelObserver &VehicleObserver() const
  {
    return this->vehicleObserver;
  }

  protected: void OnPreUpdate(
    const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &_ecm) override
  {
    this->vehicleManipulator.Update(_ecm);
  }

  protected: void OnPostUpdate(
    const ignition::gazebo::UpdateInfo &,
    const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    this->vehicleObserver.Update(_ecm);
  }

  private: ModelManipulator vehicleManipulator;

  private: ModelObserver vehicleObserver;

  private: std::string vehicleName;
};

/// A test fixture with a single vehicle for vehicle command testing.
class VehicleCommandTestFixture : public TestFixtureWithVehicle
{
  /// Constructor.
  /// \param[in] _worldName Base name of the world SDF,
  /// to be found among this package's ``worlds``.
  /// \param[in] _vehicleName Name of the vehicle model.
  public: VehicleCommandTestFixture(
      const std::string &_worldName,
      const std::string &_vehicleName)
    : TestFixtureWithVehicle(_worldName, _vehicleName)
  {
    using lrauv_ignition_plugins::msgs::LRAUVCommand;
    const std::string topicName = "/" + _vehicleName + "/command_topic";
    this->commandPublisher = this->node.Advertise<LRAUVCommand>(topicName);
  }

  /// Returns a vehicle command publisher.
  public: ignition::transport::Node::Publisher &CommandPublisher()
  {
    return this->commandPublisher;
  }

  protected: ignition::transport::Node &Node() { return this->node; }

  private: ignition::transport::Node node;

  private: ignition::transport::Node::Publisher commandPublisher;
};

/// A test fixture with a single vehicle for vehicle state testing.
class VehicleStateTestFixture : public VehicleCommandTestFixture
{
  using LRAUVState = lrauv_ignition_plugins::msgs::LRAUVState;

  /// Constructor.
  /// \param[in] _worldName Base name of the world SDF,
  /// to be found among this package's ``worlds``.
  /// \param[in] _vehicleName Name of the vehicle model.
  public: VehicleStateTestFixture(
      const std::string &_worldName,
      const std::string &_vehicleName)
    : VehicleCommandTestFixture(_worldName, _vehicleName)
  {
    const std::string topicName = "/" + _vehicleName + "/state_topic";
    this->stateSubscription.Subscribe(this->Node(), topicName, 1);
  }

  /// Returns a vehicle state subscription.
  public: Subscription<LRAUVState> &StateSubscription()
  {
    return this->stateSubscription;
  }

  private: Subscription<LRAUVState> stateSubscription;
};

}

#endif // LRAUV_SYSTEM_TESTS__TEST_FIXTURE_HH
