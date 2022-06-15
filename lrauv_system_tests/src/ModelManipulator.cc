#include "lrauv_system_tests/ModelManipulator.hh"

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/math/Pose3.hh>


namespace lrauv_system_tests
{
ModelManipulator::ModelManipulator(const std::string &_modelName)
    : modelName(_modelName)
{
}

void ModelManipulator::Update(
    gz::sim::EntityComponentManager &_ecm)
{
  if (this->poseRequest.has_value()) {
    gz::sim::World world(gz::sim::worldEntity(_ecm));
    gz::sim::Model model(
        world.ModelByName(_ecm, this->modelName));
    model.SetWorldPoseCmd(_ecm, this->poseRequest.value());
    this->poseRequest.reset();
  }
}

void ModelManipulator::SetOrientation(
    const gz::math::Quaterniond &_orientation)
{
  this->poseRequest = gz::math::Pose3d(
      gz::math::Vector3d::Zero, _orientation);
}
}
