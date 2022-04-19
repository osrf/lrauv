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
  const std::string &_baseLinkName,
  size_t _historyDepth)
: modelName(_modelName),
  baseLinkName(_baseLinkName),
  historyDepth(_historyDepth)
{
}

void ModelObserver::Update(
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  const ignition::gazebo::Entity worldEntity =
      ignition::gazebo::worldEntity(_ecm);
  ignition::gazebo::World world(worldEntity);

  const ignition::gazebo::Entity modelEntity =
      world.ModelByName(_ecm, this->modelName);
  if (ignition::gazebo::kNullEntity != modelEntity)
  {
    this->poses.push_back(
        ignition::gazebo::worldPose(modelEntity, _ecm));
    if (this->historyDepth > 0u)
    {
      if (this->poses.size() > this->historyDepth)
      {
        this->poses.pop_front();
      }
    }

    auto coordinates =
        ignition::gazebo::sphericalCoordinates(modelEntity, _ecm);
    if (coordinates.has_value())
    {
      this->sphericalCoordinates.push_back(coordinates.value());
      if (this->historyDepth > 0u)
      {
        if (this->sphericalCoordinates.size() > this->historyDepth)
        {
          this->sphericalCoordinates.pop_front();
        }
      }
    }

    ignition::gazebo::Model model(modelEntity);
    const ignition::gazebo::Entity linkEntity =
        model.LinkByName(_ecm, this->baseLinkName);
    ASSERT_NE(ignition::gazebo::kNullEntity, linkEntity);
    ignition::gazebo::Link link(linkEntity);

    auto linearVelocity = link.WorldLinearVelocity(_ecm);
    if (linearVelocity.has_value())
    {
      this->linearVelocities.push_back(linearVelocity.value());
      if (this->historyDepth > 0u)
      {
        if(this->linearVelocities.size() > this->historyDepth)
        {
          this->linearVelocities.pop_front();
        }
      }
    }

    auto angularVelocity = link.WorldAngularVelocity(_ecm);
    if (angularVelocity.has_value())
    {
      this->angularVelocities.push_back(angularVelocity.value());
      if (this->historyDepth > 0u)
      {
        if (this->angularVelocities.size() > this->historyDepth)
        {
          this->angularVelocities.pop_front();
        }
      }
    }
  }
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
