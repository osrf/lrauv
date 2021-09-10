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

#include <algorithm>
#include <random>

namespace tethys
{
/// \brief A simple acoustic model that handles incoming packets and their
/// delays. Doesn't use any fancy datastructure (it probably should). Basically
/// applies a delay and a drop-out rate to incoming messages. The dropout rate
/// is linearly proportional to the distance.
class SimpleAcousticModel : public ICommsModel
{
  /// \brief The set of messages queued. A smarter way to solve this would be to
  /// use a priority queue ordered by distance from arriving packet.
  private: std::vector<CommsPacket> packets;

  private: uint32_t address;

  private: double maxRange;

  private: double speedOfSound;

  private: std::random_device rd;  // Will be used to obtain a seed for the random number engine
    
  private: std::mt19937 gen;

  private: std::uniform_real_distribution<> uniformDistribution;

  public: SimpleAcousticModel() : gen(rd()), uniformDistribution(0,1)
  {

  }

  public: void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecms) override
  {
    this->address = _sdf->Get<uint32_t>("address");
    this->maxRange = _sdf->Get<uint32_t>("max_range");
    this->speedOfSound = _sdf->Get<uint32_t>("speed_of_sound");
  }

  public: void enqueue_msg(const CommsPacket &_packet) override
  {
    // For now don't care about packet collision
    if (_packet.To() == this->address)
      packets.push_back(_packet);
  }

  public: bool dropPacket(double distance)
  {
    if (distance > this->maxRange) return true;

    // Linear dropoff rate for now
    // auto cutoff = distance/this->maxRange; 
    // auto val = this->uniformDistribution(gen);

    return false;
  }

  public: void step(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm,
    MessageManager &_messageMgr,
    const ignition::math::Pose3d &_pose) override
  {
    this->packets.erase(std::remove_if(
      this->packets.begin(), this->packets.end(), 
      [_info, _pose, &_messageMgr, self = this]
      (const auto packet) -> bool const
      {
        auto distToTransmitter = packet.Position().Distance(_pose.Pos());
        std::chrono::steady_clock::time_point currTime(_info.simTime);
        
        auto duration = 
          std::chrono::duration_cast<std::chrono::duration<double>>(
            currTime - packet.TimeOfTransmission());
        auto distanceCoveredByMessage = duration.count() * self->speedOfSound; 
        
        if (distToTransmitter <= distanceCoveredByMessage)
        {
          if (!self->dropPacket(distToTransmitter))
          {
            _messageMgr.MessageReachedDestination(packet);
          }
          return true;
        }
        return false;
    }), this->packets.end());
  }
};

IGNITION_ADD_PLUGIN(SimpleAcousticModel,
    SimpleAcousticModel::ICommsModel)
} 
