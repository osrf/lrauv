#include "lrauv_system_tests/ModelManipulator.hh"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/math/Pose3.hh>


namespace lrauv_system_tests
{
ModelManipulator::ModelManipulator(const std::string &_modelName)
    : modelName(_modelName)
{
}

void ModelManipulator::Update(
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->poseRequest.has_value()) {
    ignition::gazebo::World world(
        ignition::gazebo::worldEntity(_ecm));
    ignition::gazebo::Model model(
        world.ModelByName(_ecm, this->modelName));
    model.SetWorldPoseCmd(_ecm, this->poseRequest.value());
    this->poseRequest.reset();
  }
}

void ModelManipulator::SetOrientation(
    const ignition::math::Quaterniond &_orientation)
{
  this->poseRequest = ignition::math::Pose3d(
      ignition::math::Vector3d::Zero, _orientation);
}
}
