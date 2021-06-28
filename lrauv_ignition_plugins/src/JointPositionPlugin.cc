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

#include "JointPositionPlugin.hh"

namespace tethys
{
class TethysJointPrivateData
{
  public: double maxVelocity{0.07};

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
  if (topic.empty())
  {
    ignerr << "Failed to create topic for joint [" << this->dataPtr->jointName
          << "]" << std::endl;
    return;
  }
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
}

//////////////////////////////////////////////////
void TethysJointPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
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

  // Get command position
  double cmdPos;
  {
    std::lock_guard<std::mutex> lock(dataPtr->jointCmdMutex);
    cmdPos = dataPtr->jointPosCmd;
  }

  std::chrono::duration<double, std::ratio<1,1>> delta_t = _info.dt;
  auto maxPositionChange = delta_t.count() * dataPtr->maxVelocity;

  double desiredVelocity = 0;
  if (cmdPos < currentPos[dataPtr->jointIndex])
  {
    if (currentPos[dataPtr->jointIndex] - cmdPos > maxPositionChange)
    {
      desiredVelocity = 
        (cmdPos - currentPos[dataPtr->jointIndex])/delta_t.count();
    }
    else
    {
      desiredVelocity = -dataPtr->maxVelocity;
    }
  }
  else if (cmdPos > currentPos[dataPtr->jointIndex])
  {
    if (cmdPos - currentPos[dataPtr->jointIndex] > maxPositionChange)
    {
      desiredVelocity =
        (cmdPos - currentPos[dataPtr->jointIndex])/delta_t.count();
    }
    else
    {
      desiredVelocity = dataPtr->maxVelocity;
    }
  }

  // Set Command velocity
  auto velocityComp =
    _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(this->dataPtr->jointEntity);
  if (velocityComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity,
      ignition::gazebo::components::JointVelocityCmd({desiredVelocity}));
  }
  else
  {
    velocityComp->Data()[dataPtr->jointIndex] = desiredVelocity;
  }
}
}

IGNITION_ADD_PLUGIN(
  tethys::TethysJointPlugin,
  ignition::gazebo::System,
  tethys::TethysJointPlugin::ISystemConfigure,
  tethys::TethysJointPlugin::ISystemPreUpdate)
