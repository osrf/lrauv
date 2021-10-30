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

#ifndef __LRAUV_IGNITION_PLUGINS_COMMS_COMMSMODEL_HH__
#define __LRAUV_IGNITION_PLUGINS_COMMS_COMMSMODEL_HH__

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>

#include "CommsPacket.hh"
#include "MessageManager.hh"

namespace tethys
{
/////////////////////////////////////
/// \brief Abstract interface to define how the environment should handle
/// wireless communication simulation. This class should be responsible for
/// handling dropouts, decay and packet collisions.
class ICommsModel
{
  /// \brief This method is called during setup.
  /// \param[in] _entity - The model you are bound to.
  /// \param[in] _sdf - The SDF description of the plugin.
  /// \param[in] _ecm - Ignition's Entity Component Manager.
  public: virtual void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm) = 0;

  /// \brief This method is called when the message bus delivers a message.
  /// You should override this method to determine when a message is coming in.
  /// \param[in] _packet - Incoming comms packet.
  public: virtual void EnqueueMsg(const CommsPacket &_packet) = 0;

  /// \brief This method is called when there is a timestep in the simulator
  /// override this to update your data structures as needed.
  /// \param[in] _info - Simulator information about the current timestep.
  /// \param[in] _ecm - Ignition's ECM.
  /// \param[in] _messageMgr - Use this to mark the message as arrived.
  /// \param[in] _pose - The current pose of the receiving model.
  public: virtual void Step(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm,
    MessageManager &_messageMgr,
    const ignition::math::Pose3d &_pose) = 0;
  
  /// \brief Destructor
  public: virtual ~ICommsModel() = default;
};
}

#endif
