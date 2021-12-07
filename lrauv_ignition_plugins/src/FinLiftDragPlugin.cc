/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "FinLiftDragPlugin.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components.hh>

#include <map>

#include <Eigen/Eigen>


namespace tethys
{
class FinLiftDragPrivateData
{
  private: std::map<double, double> splineCd;

  private: std::map<double, double> splineCl;

  private: double interpolate(
    const std::map<double, double> &_spline,
    const double _angle)
  {
    // For now use simple linear interpolation
    if (_spline.count(_angle) > 0)
    {
      return _spline.find(_angle)->second;
    }

    double rightVal, rightAng;
    auto right = _spline.upper_bound(_angle);
    if (right == _spline.end())
    {
      // Wrap around
      rightVal = _spline.begin()->second;
      rightAng = _spline.begin()->first;
    }
    else
    {
      rightVal = right->second;
      rightAng = right->first;
    }

    double leftVal, leftAng;
    if (right == _spline.begin())
    {
      leftVal = _spline.rbegin()->second;
      leftAng = _spline.rbegin()->first;
    }
    else {
      auto left = std::prev(right);
      leftVal = left->second;
      leftAng = left->first;
    }
  }

  public: float CalcDragCoeff(float _angle)
  {
    return 300;
  }

  public: float CalcLiftCoeff(float _angle)
  {
    return 0.1;
  }

  /// \brief Create a spline using sdf parameters.
  private: std::map<double, double> createSpline(
    const std::shared_ptr<const sdf::Element> &_sdf, std::string _sdfParam)
  {
    if (!_sdf->HasElement(_sdfParam))
    {
      ignerr << "No " << _sdfParam << " specified" << std::endl;
      this->validConfig = false;
      return {};
    }

    auto elem = _sdf->GetFirstElement();
    if (elem == nullptr)
    {
      ignerr << "Unable to get element description" << std::endl;
      this->validConfig = false;
      return {};
    }

    while (elem != nullptr && elem->GetName() != _sdfParam)
    {
      // Search for name
      elem = elem->GetNextElement();
    }

    if (elem == nullptr)
    {
      this->validConfig = false;
      //Dead branch cause we already check in lin 62
      return {};
    }

    auto childEntry = elem->GetFirstElement();
    std::map<double, double> points;
    while (childEntry != nullptr)
    {
      if (childEntry->GetName() == "spline_point")
      {
        double angleOfAttack = childEntry->Get<double>("angle");
        double coeff = childEntry->Get<double>("coeff");
        points.insert({angleOfAttack, coeff});
      }
      childEntry = childEntry->GetNextElement();
    }

    return points;
  }

  public: void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm)
  {
    this->model = ignition::gazebo::Model(_entity);
    this->splineCd = createSpline(_sdf, "drag_coeffs");
    ConfigureLink(_ecm, _sdf);
    this->fluidDensity = _sdf->Get<double>("fluid_density");
    this->finArea = _sdf->Get<double>("area");
  }

  private: void ConfigureLink(
    ignition::gazebo::EntityComponentManager &_ecm,
    const std::shared_ptr<const sdf::Element> &_sdf)
  {
    if (_sdf->HasElement("link_name"))
    {
      auto linkName = _sdf->Get<std::string>("link_name");
      auto entities =
        ignition::gazebo::entitiesFromScopedName(
          linkName, _ecm, this->model.Entity());

      if (entities.empty())
      {
        ignerr << "Link with name[" << linkName << "] not found. "
              << "The LiftDrag will not generate forces\n";
        this->validConfig = false;
        return;
      }
      else if (entities.size() > 1)
      {
        ignwarn << "Multiple link entities with name[" << linkName << "] found."
              << "Using the first one.\n";
      }

      this->linkEntity = *entities.begin();
      if (!_ecm.EntityHasComponentType(this->linkEntity,
                                      ignition::gazebo::components::Link::typeId))
      {
        this->linkEntity = ignition::gazebo::kNullEntity;
        ignerr << "Entity with name[" << linkName << "] is not a link\n";
        this->validConfig = false;
        return;
      }

      ignition::gazebo::Link link(this->linkEntity);
      link.EnableVelocityChecks(_ecm);
    }
    else
    {
      ignerr << "The LiftDrag system requires the 'link_name' parameter\n";
      this->validConfig = false;
      return;
    }
  }

  /// \brief Returns a pair consisting of angle of attack and velocity.
  public: std::pair<double, double> GetAngleOfAttackAndVelocity(
    ignition::gazebo::EntityComponentManager &_ecm)
  {
    ignition::gazebo::Link link(this->linkEntity);
    auto linVel = link.WorldLinearVelocity(_ecm);
    if (!linVel.has_value())
    {
      ignerr << "could not get velocity of " <<
        link.Name(_ecm).value() << std::endl;
      return {0, 0};
    }
    auto pose = link.WorldPose(_ecm);
    if (!pose.has_value())
    {
      ignerr << "could not get velocity of " <<
        link.Name(_ecm).value() << std::endl;
      return {0, 0};
    }

    // Get velocity in local frame
    auto localVel = pose.value().Rot().Inverse() * linVel.value();

    // Project velocity into lift-drag plane
    auto liftDragPlaneNormal = this->forward.Cross(this->upward);
    auto normalProjection = liftDragPlaneNormal.Dot(localVel) /
      (liftDragPlaneNormal.Length() * liftDragPlaneNormal.Length());
    auto velInLDPlane = localVel - normalProjection;

    // Determine Angle of Attack
    auto angleOfAttack = acos(velInLDPlane.Dot(this->forward) /
      (velInLDPlane.Length() * this->forward.Length()));

    // Determine the sign by testing against the upward vector
    if (velInLDPlane.Dot(this->upward) < 0)
      angleOfAttack *= -1;

    auto velocity = velInLDPlane.Length();
    return {angleOfAttack, velocity};
  }

  /// \brief Get world velocity
  public: ignition::math::Vector3d WorldLinearVelocity(
    ignition::gazebo::EntityComponentManager &_ecm)
  {
    ignition::gazebo::Link link(this->linkEntity);
    auto linVel = link.WorldLinearVelocity(_ecm);
    if (!linVel.has_value())
    {
      ignerr << "could not get velocity of " <<
        link.Name(_ecm).value() << std::endl;
      return {0, 0, 0};
    }
    return linVel.value();
  }

  public: void ApplyWorldForce(
    ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::math::Vector3d &_force)
  {
    ignition::gazebo::Link link(this->linkEntity);
    link.AddWorldWrench(_ecm, _force, {0,0,0});

    ignerr << _force << "\n";
  }

  /// \brief Normally, this is taken as a direction parallel to the chord
  /// of the airfoil in zero angle of attack forward flight.
  public: ignition::math::Vector3d forward = - ignition::math::Vector3d::UnitX;

  /// \brief A vector in the lift/drag plane, perpendicular to the forward
  /// vector. Inflow velocity orthogonal to forward and upward vectors
  /// is considered flow in the wing sweep direction.
  public: ignition::math::Vector3d upward = ignition::math::Vector3d::UnitZ;

  public: ignition::gazebo::Entity linkEntity;

  public: ignition::gazebo::Model model;

  public: double fluidDensity{1000};

  public: double finArea{0.0042};

  public: bool validConfig {true};

};

FinLiftDragPlugin::FinLiftDragPlugin() :
  dataPtr(std::make_unique<FinLiftDragPrivateData>())
{

}

void FinLiftDragPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  ignerr << "loading FinDrag plugin" << std::endl;
  this->dataPtr->Configure(_entity, _sdf, _ecm);
}

void FinLiftDragPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused || !this->dataPtr->validConfig)
    return;
  auto [aoa, vel] = this->dataPtr->GetAngleOfAttackAndVelocity(_ecm);

  // Drag Equation
  auto drag =
    0.5 * this->dataPtr->CalcDragCoeff(aoa) * vel * vel * this->dataPtr->finArea;

  // Drag acts in the direction of motion
  auto worldVel = this->dataPtr->WorldLinearVelocity(_ecm);

  // Get the drag vector and add it.
  auto directionVector = - worldVel.Normalized();

  // Apply the drag force
  this->dataPtr->ApplyWorldForce(_ecm, directionVector * drag);

  // Drag Equation
  auto lift =
    0.5 * this->dataPtr->CalcLiftCoeff(aoa) * vel * vel * this->dataPtr->finArea;

  // Lift is 90 degrees to the 

}
}

IGNITION_ADD_PLUGIN(
  tethys::FinLiftDragPlugin,
  ignition::gazebo::System,
  tethys::FinLiftDragPlugin::ISystemConfigure,
  tethys::FinLiftDragPlugin::ISystemPreUpdate)
