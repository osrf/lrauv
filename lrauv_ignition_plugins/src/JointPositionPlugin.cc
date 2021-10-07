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

#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

#include "JointPositionPlugin.hh"

namespace tethys
{
class TethysJointPrivateData
{
  /// \brief tThe max velocity at which the joint can move.
  public: double maxVelocity{0.0007};

  /// \brief Callback for position subscription
  /// \param[in] _msg Position message
  public: void OnCmdPos(const ignition::msgs::Double &_msg);

  /// \brief Ignition communication node.
  public: ignition::transport::Node node;

  /// \brief Joint Entity
  public: ignition::gazebo::Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Commanded joint position
  public: double jointPosCmd {0};

  /// \brief Model interface
  public: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

  /// \brief mutex to protect joint commands
  public: std::mutex jointCmdMutex;

  /// \brief Joint index to be used.
  public: unsigned int jointIndex = 0u;

};

//////////////////////////////////////////////////
void TethysJointPrivateData::OnCmdPos(const ignition::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointCmdMutex);
  this->jointPosCmd = _msg.data();
}

//////////////////////////////////////////////////
TethysJointPlugin::TethysJointPlugin():
  dataPtr(std::make_unique<TethysJointPrivateData>())
{
  // do nothing
}

//////////////////////////////////////////////////
void TethysJointPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = ignition::gazebo::Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "JointPositionController plugin should be attached to a model "
          << "entity. Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  this->dataPtr->jointName = _sdf->Get<std::string>("joint_name");

  if (this->dataPtr->jointName == "")
  {
    ignerr << "JointPositionController found an empty jointName parameter. "
          << "Failed to initialize.";
    return;
  }

  if (_sdf->HasElement("joint_index"))
  {
    this->dataPtr->jointIndex = _sdf->Get<unsigned int>("joint_index");
  }

  std::string topic =
    ignition::transport::TopicUtils::AsValidTopic("/model/" +
    this->dataPtr->model.Name(_ecm) + "/joint/" + this->dataPtr->jointName +
    "/" + std::to_string(this->dataPtr->jointIndex) + "/cmd_pos");

  if (_sdf->HasElement("topic"))
  {
    topic = ignition::transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      ignerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
            << "]" << " for joint [" << this->dataPtr->jointName
            << "]" << std::endl;
      return;
    }
  }
  this->dataPtr->node.Subscribe(
      topic, &TethysJointPrivateData::OnCmdPos, this->dataPtr.get());
  ignmsg << "Listening to commands on [" << topic << "]" << std::endl;
}

//////////////////////////////////////////////////
void TethysJointPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  // Access joint entity
  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);

    if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity)
    {
      static bool informed{false};
      if (!informed)
      {
        ignerr << "Failed to find joint [" << this->dataPtr->jointName
               << "] in model [" << this->dataPtr->model.Name(_ecm) << "]"
               << std::endl;
        informed = true;
      }
    }

    // Get joint velocity limit specified in SDF
    auto jointAxisComp =
      _ecm.Component<ignition::gazebo::components::JointAxis>(
        this->dataPtr->jointEntity);
    if (jointAxisComp != nullptr)
    {
      // Access max velocity in joint SDF
      this->dataPtr->maxVelocity = jointAxisComp->Data().MaxVelocity();
    }
  }

  if (_info.paused)
    return;

  // Check joint position component
  auto jointPosComp =
    _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->jointEntity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity, ignition::gazebo::components::JointPosition());
  }

  if (jointPosComp == nullptr)
    return;

  auto currentPos = jointPosComp->Data();
  if (currentPos.size() == 0)
  {
    ignerr << "Joint ["
      << this->dataPtr->jointName
      << "] unable to get position" <<std::endl;
    return;
  }

  // Get command position
  double cmdPos;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointCmdMutex);
    cmdPos = this->dataPtr->jointPosCmd;
  }

  std::chrono::duration<double, std::ratio<1,1>> deltaT = _info.dt;
  auto maxPositionChange = deltaT.count() * this->dataPtr->maxVelocity;

  double desiredVelocity = 0;
  if (cmdPos < currentPos[this->dataPtr->jointIndex])
  {
    if (currentPos[this->dataPtr->jointIndex] - cmdPos > maxPositionChange)
    {
      // If the gap between the current position and the command position is
      // more than the max change in one clock cycle then we want to move the
      // joint at the max velocity.
      desiredVelocity = -this->dataPtr->maxVelocity;
    }
    else
    {
      // Otherwise if the joint gap is less than the max change in one clock
      // cycle, move it to the correct position within the clock cycle to
      // prevent overshoot.
      desiredVelocity =
        (cmdPos - currentPos[this->dataPtr->jointIndex])/deltaT.count();
    }
  }
  else if (cmdPos > currentPos[this->dataPtr->jointIndex])
  {
    if (cmdPos - currentPos[this->dataPtr->jointIndex] > maxPositionChange)
    {
      // If the gap between the current position and the command position is
      // more than the max change in one clock cycle then we want to move the
      // joint at the max velocity.
      desiredVelocity = this->dataPtr->maxVelocity;
    }
    else
    {
      // Otherwise if the joint gap is less than the max change in one clock
      // cycle, move it to the correct position within the clock cycle to
      // prevent overshoot.
      desiredVelocity =
        (cmdPos - currentPos[this->dataPtr->jointIndex])/deltaT.count();
    }
  }

  // Set Command velocity
  auto velocityCmdComp =
    _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
      this->dataPtr->jointEntity);
  if (velocityCmdComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity,
      ignition::gazebo::components::JointVelocityCmd({desiredVelocity}));
  }
  else
  {
    auto velocityCmd = velocityCmdComp->Data();
    std::vector<double> velocityCmdNew = std::vector<double>(velocityCmd);
    velocityCmdNew[this->dataPtr->jointIndex] = desiredVelocity;
    velocityCmdComp->SetData(velocityCmdNew,
      [](const std::vector<double> &v1, const std::vector<double> &v2) -> bool
      {
        return (v1 == v2);
      });
  }
}
}

IGNITION_ADD_PLUGIN(
  tethys::TethysJointPlugin,
  ignition::gazebo::System,
  tethys::TethysJointPlugin::ISystemConfigure,
  tethys::TethysJointPlugin::ISystemPreUpdate)
