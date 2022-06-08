#include "lrauv_system_tests/ModelObserver.hh"

#include <gtest/gtest.h>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

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
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  const gz::sim::Entity worldEntity =
      gz::sim::worldEntity(_ecm);
  gz::sim::World world(worldEntity);

  const gz::sim::Entity modelEntity =
      world.ModelByName(_ecm, this->modelName);
  if (gz::sim::kNullEntity != modelEntity)
  {
    this->times.push_back(_info.simTime);

    this->poses.push_back(
        gz::sim::worldPose(modelEntity, _ecm));

    auto coordinates =
        gz::sim::sphericalCoordinates(modelEntity, _ecm);
    this->sphericalCoordinates.push_back(
        coordinates.value_or(gz::math::Vector3d::NaN));

    gz::sim::Model model(modelEntity);
    const gz::sim::Entity linkEntity =
        model.LinkByName(_ecm, this->baseLinkName);
    ASSERT_NE(gz::sim::kNullEntity, linkEntity);
    gz::sim::Link link(linkEntity);

    auto linearVelocity = link.WorldLinearVelocity(_ecm);
    this->linearVelocities.push_back(
        linearVelocity.value_or(gz::math::Vector3d::NaN));

    auto angularVelocity = link.WorldAngularVelocity(_ecm);
    this->angularVelocities.push_back(
        angularVelocity.value_or(gz::math::Vector3d::NaN));

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

const std::deque<gz::math::Pose3d> &
ModelObserver::Poses() const
{
  return this->poses;
}

const std::deque<gz::math::Vector3d> &
ModelObserver::SphericalCoordinates() const
{
  return this->sphericalCoordinates;
}

const std::deque<gz::math::Vector3d> &
ModelObserver::LinearVelocities() const
{
  return this->linearVelocities;
}

const std::deque<gz::math::Vector3d> &
ModelObserver::AngularVelocities() const
{
  return this->angularVelocities;
}

}
