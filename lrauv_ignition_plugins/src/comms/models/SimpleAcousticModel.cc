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

/* Development of this module has been funded by the Monterey Bay Aquarium
Research Institute (MBARI) and the David and Lucile Packard Foundation */

#include <lrauv_ignition_plugins/comms/CommsModel.hh>
#include <lrauv_ignition_plugins/comms/CommsPacket.hh>
#include <ignition/plugin/Register.hh>

namespace tethys
{
/// \brief A simple acoustic model that handles incoming packets and their
/// delays
class SimpleAcousticModel : public ICommsModel
{
  //private: std::vector<CommsPacket> _packet;

  public: void enqueue_msg(const CommsPacket &_packet) override
  {
    
  }

  /// \brief This method is called when the message bus 
  public: void step(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm,
    MessageManager &_messageMgr) override
  {

  }
  
};

IGNITION_ADD_PLUGIN(SimpleAcousticModel,
    SimpleAcousticModel::ICommsModel)
} 
