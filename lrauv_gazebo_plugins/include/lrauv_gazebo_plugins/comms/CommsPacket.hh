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

#ifndef __LRAUV_IGNITION_PLUGINS_COMMS_COMMSPACKET_HH__
#define __LRAUV_IGNITION_PLUGINS_COMMS_COMMSPACKET_HH__

#include <chrono>
#include <memory>

#include <gz/math/Vector3.hh>

#include "lrauv_gazebo_plugins/lrauv_acoustic_message.pb.h"
#include "lrauv_gazebo_plugins/lrauv_internal_comms.pb.h"

namespace tethys
{

//////////////////////////////////////////////////
class CommsPacket
{
  /// \brief Enum for the message type
  public: enum class MsgType {
    /// \brief Range requests
    RANGE_REQUEST,
    /// \brief Response to range request
    RANGE_RESPONSE,
    /// \brief Data
    DATA
  };
  
  /// \brief Returns the packet type.
  public: MsgType Type() const;

  /// \brief The position from which the transmission is made
  public: gz::math::Vector3d Position() const;

  /// \brief Time of transmission
  public: std::chrono::steady_clock::time_point TimeOfTransmission() const;

  /// \brief The target vehicle
  public: uint32_t To() const;

  /// \brief The sending vehicle
  public: uint32_t From() const;

  /// \brief The data in the message
  public: std::string Data() const;

  /// \brief External comms data which you interface with
  public: lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage ToExternalMsg() const;
  
  /// \brief Internal comms data which you don't interface with
  public: lrauv_gazebo_plugins::msgs::LRAUVInternalComms ToInternalMsg() const;

  class CommsPacketPrivateData;
  /// \brief pimpl
  private: std::shared_ptr<CommsPacketPrivateData> dataPtr;

  /// \brief Factory method
  /// \param[in] _datapayload - incoming message to be sent without position and
  /// transmission time.
  /// \param[in] _position - position of message.
  /// \param[in] _timeOfTx - time at which message is transmitted.
  public: static CommsPacket make(
    const lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage &_datapayload,
    const gz::math::Vector3d _position,
    const std::chrono::steady_clock::time_point _timeOfTx
    );

  /// \brief Factory method
  public: static CommsPacket make(
    const lrauv_gazebo_plugins::msgs::LRAUVInternalComms &_datapayload);

  /// \brief Default destructor
  public: ~CommsPacket();

  /// \brief Constructor. Private: use the factory methods.
  private: CommsPacket();

  /// \brief Comparator for simplicity.
  public: bool operator==(const CommsPacket& other) const;
};
}
#endif
