#include "lrauv_system_tests/ModelObserver.hh"

#include <gtest/gtest.h>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>

namespace lrauv_system_tests
{
ModelObserver::ModelObserver(
  const std::string &_modelName,
  const std::string &_baseLinkName)
: modelName(_modelName),
  baseLinkName(_baseLinkName)
{
}

void ModelObserver::LimitTo(
  std::chrono::steady_clock::duration _windowSize)
{
  this->windowSize = _windowSize;
}

void ModelObserver::Update(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  const ignition::gazebo::Entity worldEntity =
      ignition::gazebo::worldEntity(_ecm);
  ignition::gazebo::World world(worldEntity);

  const ignition::gazebo::Entity modelEntity =
      world.ModelByName(_ecm, this->modelName);
  if (ignition::gazebo::kNullEntity != modelEntity)
  {
    this->times.push_back(_info.simTime);

    this->poses.push_back(
        ignition::gazebo::worldPose(modelEntity, _ecm));

    auto coordinates =
        ignition::gazebo::sphericalCoordinates(modelEntity, _ecm);
    this->sphericalCoordinates.push_back(
        coordinates.value_or(ignition::math::Vector3d::NaN));

    ignition::gazebo::Model model(modelEntity);
    const ignition::gazebo::Entity linkEntity =
        model.LinkByName(_ecm, this->baseLinkName);
    ASSERT_NE(ignition::gazebo::kNullEntity, linkEntity);
    ignition::gazebo::Link link(linkEntity);

    auto linearVelocity = link.WorldLinearVelocity(_ecm);
    this->linearVelocities.push_back(
        linearVelocity.value_or(ignition::math::Vector3d::NaN));

    auto angularVelocity = link.WorldAngularVelocity(_ecm);
    this->angularVelocities.push_back(
        angularVelocity.value_or(ignition::math::Vector3d::NaN));

    if (this->windowSize != std::chrono::steady_clock::duration::zero())
    {
      while (this->times.back() - this->times.front() > this->windowSize)
      {
        this->times.pop_front();
        this->poses.pop_front();
        this->sphericalCoordinates.pop_front();
        this->linearVelocities.pop_front();
        this->angularVelocities.pop_front();
      }
    }
  }
}

const std::deque<std::chrono::steady_clock::duration> &
ModelObserver::Times() const
{
  return this->times;
}

const std::deque<ignition::math::Pose3d> &
ModelObserver::Poses() const
{
  return this->poses;
}

const std::deque<ignition::math::Vector3d> &
ModelObserver::SphericalCoordinates() const
{
  return this->sphericalCoordinates;
}

const std::deque<ignition::math::Vector3d> &
ModelObserver::LinearVelocities() const
{
  return this->linearVelocities;
}

const std::deque<ignition::math::Vector3d> &
ModelObserver::AngularVelocities() const
{
  return this->angularVelocities;
}

}
