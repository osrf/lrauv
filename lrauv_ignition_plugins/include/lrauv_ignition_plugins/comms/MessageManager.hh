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

#ifndef __LRAUV_IGNITION_PLUGINS_COMMS_MESSAGEMGR_HH__
#define __LRAUV_IGNITION_PLUGINS_COMMS_MESSAGEMGR_HH__

#include "CommsPacket.hh"

#include <ignition/transport/Node.hh>
namespace tethys
{
//////////////////////////////////////////////////
class MessageManager
{
  ////////////////////////////////////////////////
  /// \brief Call this method when you want to declare that the message
  /// has reached its destination.
  public: void MessageReachedDestination(const CommsPacket& packet)
  {
    this->externalPublisher.Publish(packet.ToExternalMsg());
  }
  ////////////////////////////////////////////////
  public: ignition::transport::Node::Publisher externalPublisher;
};
}
#endif
