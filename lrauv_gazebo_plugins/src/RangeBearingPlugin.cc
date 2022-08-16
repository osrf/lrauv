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

#include "RangeBearingPlugin.hh"

#include "lrauv_gazebo_plugins/lrauv_range_bearing_request.pb.h"
#include "lrauv_gazebo_plugins/lrauv_range_bearing_response.pb.h"

#include <queue>

#include <gz/math/Matrix4.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <lrauv_gazebo_plugins/comms/CommsClient.hh>

namespace tethys
{
////////////////////////////////////////////////
struct IncomingRangePing
{
  uint32_t from;
  uint32_t reqId;
  std::chrono::steady_clock::time_point timeOfReception;
};
////////////////////////////////////////////////
class RangeBearingPrivateData
{
  /// \brief Topic to listen on for incomming requests
  public: std::string topicPrefix {"/range_bearing/"};

  /// \brief Comms client for handling
  public: std::shared_ptr<CommsClient> commsClient;

  /// \brief Bind topics of interest
  public: void BindToAddress(const uint32_t address);

  /// \brief Callback for CommsClient
  public: void OnReceiveCommsMsg(
    const lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage& message);

  /// \brief Callback for range requests
  public: void OnRangeRequest(
    const lrauv_gazebo_plugins::msgs::LRAUVRangeBearingRequest& req);

  /// \brief Publish range-bearing response
  public: void PublishResponse(
    const lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage& resp);

  /// \brief Queue which we need to respond to.
  public: std::queue<IncomingRangePing> messageQueue;

  /// \brief Table to lookup outgoing requests
  /// TODO(arjo) Add a garbage collection mechanism for packets that
  /// are lost.
  public: std::unordered_map<uint32_t,
    std::chrono::steady_clock::time_point> transmissionTime;

  /// \brief Transport node
  public: gz::transport::Node node;

  /// \brief Publisher for results
  public: gz::transport::Node::Publisher pub;

  /// \brief Processing duration
  public: std::chrono::steady_clock::duration processingDelay;

  /// \brief This object's address
  public: uint32_t address;

  /// \brief Current time
  public: std::chrono::steady_clock::time_point timeNow;

  /// \brief Link and entity which this is bound to
  public: gz::sim::Entity linkEntity;

  /// \brief The current pose
  public: gz::math::Pose3d currentPose;

  /// \brief Speed of sound. Units: m/s
  public: double speedOfSound {15000};

  /// \brief mutex
  public: std::mutex mtx;
};

////////////////////////////////////////////////
void RangeBearingPrivateData::BindToAddress(const uint32_t address)
{
  this->address = address;
  this->commsClient = std::make_shared<CommsClient>(
    address,
    std::bind(&RangeBearingPrivateData::OnReceiveCommsMsg, this,
      std::placeholders::_1));
}

////////////////////////////////////////////////
void RangeBearingPrivateData::OnReceiveCommsMsg(
  const lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage& message)
{
  using MsgType =
    lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage::MessageType;

  lrauv_gazebo_plugins::msgs::LRAUVRangeBearingRequest req;
  std::istringstream stream{message.data()};
  req.ParseFromIstream(&stream);
  IncomingRangePing ping{message.from(), req.req_id(), this->timeNow};

  switch(message.type())
  {
  case MsgType::LRAUVAcousticMessage_MessageType_RangeRequest:
    // Place range ping from other vehicle on queue
    this->messageQueue.push(ping);
    break;
  case MsgType::LRAUVAcousticMessage_MessageType_RangeResponse:
    // Publish range response
    this->PublishResponse(message);
    break;
  default:
    ignwarn << "Unable to process message type\n";
  }
}

////////////////////////////////////////////////
void RangeBearingPrivateData::OnRangeRequest(
  const lrauv_gazebo_plugins::msgs::LRAUVRangeBearingRequest& req)
{
  using Message = lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage;
  Message message;
  message.set_to(req.to());
  message.set_from(this->address);
  message.set_type(
    Message::MessageType::LRAUVAcousticMessage_MessageType_RangeRequest);
  std::ostringstream stream;
  req.SerializeToOstream(&stream);
  message.set_data(stream.str());

  std::lock_guard<std::mutex> lock(this->mtx);
  this->transmissionTime[req.req_id()] = this->timeNow;
  this->commsClient->SendPacket(message);
}

////////////////////////////////////////////////
void RangeBearingPrivateData::PublishResponse(
  const lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage& msg)
{
  lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse resp;
  std::istringstream stream{msg.data()};
  resp.ParseFromIstream(&stream);

  auto timeOfTx = transmissionTime[resp.req_id()];
  transmissionTime.erase(resp.req_id());
  auto duration = std::chrono::duration<double>(
    this->timeNow - timeOfTx - this->processingDelay);
  auto range = (this->speedOfSound * duration.count()) / 2;

  // Get current pose
  auto poseOffset = gz::math::Matrix4d(this->currentPose);
  auto otherVehiclesPos =
    gz::math::Vector3d(
      resp.bearing().x(), resp.bearing().y(), resp.bearing().z());
  // Transform pose of other vehicle to local frame
  auto poseInLocalFrame = poseOffset.Inverse() * otherVehiclesPos;

  gzdbg << "Current pose " << poseOffset.Pose().Pos() << "R: " << poseOffset.Pose().Rot().Euler() << "\n";
  gzdbg << "Target pose (global frame)" << otherVehiclesPos << "\n";
  gzdbg << "Target pose (local frame)" << poseInLocalFrame << "\n";

  // Elevation is given as a function of angle from XY plane of the vehicle 
  // with positive facing down.
  auto negativeZAxis = gz::math::Vector3d(0, 0, -1);
  auto elev =
    asin(negativeZAxis.Dot(poseInLocalFrame) / poseInLocalFrame.Length());

  // Azimuth is given by the angle from the X axis in vehicle frame.
  auto xyProj = gz::math::Vector2d(poseInLocalFrame.X(),
    poseInLocalFrame.Y());
  // TODO(arjo): This minus sign shouldn't be necessary.
  auto azimuth = (xyProj.Length() < 0.001) ? 0 : -atan2(xyProj.Y(), xyProj.X());
  gzdbg << "Elevation " << elev << " Azimuth " << azimuth << "\n";

  lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse finalAnswer;
  finalAnswer.set_range(range);
  finalAnswer.set_req_id(resp.req_id());

  gz::msgs::Vector3d* vec = new gz::msgs::Vector3d;
  vec->set_x(poseInLocalFrame.Length()); vec->set_y(elev); vec->set_z(azimuth);
  finalAnswer.set_allocated_bearing(vec);
  this->pub.Publish(finalAnswer);
}

////////////////////////////////////////////////
RangeBearingPlugin::RangeBearingPlugin():
  dataPtr(std::make_unique<RangeBearingPrivateData>())
{
  // Do nothing
}

////////////////////////////////////////////////
void RangeBearingPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/)
{
  if (!_sdf->HasElement("address"))
  {
    gzerr << "<address> tag not found" << std::endl;
    return;
  }
  this->dataPtr->BindToAddress(_sdf->Get<uint32_t>("address"));

  if (!_sdf->HasElement("processing_delay"))
  {
    gzerr << "<processing_delay> not specified." << std::endl;
    return;
  }
  this->dataPtr->processingDelay = 
    std::chrono::steady_clock::duration{
      (long)(_sdf->Get<double>("processing_delay") * 1e9)};

  if (!_sdf->HasElement("speed_of_sound"))
  {
    gzerr << "<speed_of_sound> not specified\n";
    return;
  }
  this->dataPtr->speedOfSound = _sdf->Get<double>("speed_of_sound");

  if (!_sdf->HasElement("link_name"))
  {
    gzerr <<
      "<link_name> - expected the link name of the receptor" << std::endl;
    return;
  }
  auto vehicleModel = gz::sim::Model(_entity);
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = vehicleModel.LinkByName(_ecm, linkName);
  if(this->dataPtr->linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "Link " << linkName << " was not found in "
      << vehicleModel.Name(_ecm) << std::endl;
    return;
  }
  gz::sim::enableComponent<gz::sim::components::WorldPose>(
    _ecm, this->dataPtr->linkEntity);

  if (_sdf->HasElement("namespace"))
  {
    this->dataPtr->topicPrefix = _sdf->Get<std::string>("namespace")
      + this->dataPtr->topicPrefix;
  }

  this->dataPtr->node.Subscribe(
    this->dataPtr->topicPrefix + "requests",
    &RangeBearingPrivateData::OnRangeRequest,
    this->dataPtr.get()
  );

  this->dataPtr->pub = this->dataPtr->node.Advertise<
    lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse>(
      this->dataPtr->topicPrefix + "responses");
}

////////////////////////////////////////////////
void RangeBearingPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  using MsgType =
    lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage::MessageType;

  if(_info.paused)
    return;

  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto pose = baseLink.WorldPose(_ecm);

  if (!pose.has_value())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
  this->dataPtr->currentPose = pose.value();
  this->dataPtr->timeNow = std::chrono::steady_clock::time_point{
    _info.simTime};

  // Iterate through queue to identify acoustic messages
  // that are ready to be responded to. This part implements the 300ms delay
  // that the transponder has.
  while(
    !this->dataPtr->messageQueue.empty()
    && (this->dataPtr->messageQueue.front().timeOfReception
      + this->dataPtr->processingDelay) <= this->dataPtr->timeNow)
  {
    // Handles incoming messages
    auto ping = this->dataPtr->messageQueue.front();
    lrauv_gazebo_plugins::msgs::LRAUVAcousticMessage message;
    lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse resp;
    message.set_to(ping.from);
    message.set_from(this->dataPtr->address);
    message.set_type(MsgType::LRAUVAcousticMessage_MessageType_RangeResponse);

    resp.set_req_id(ping.reqId);
    gz::msgs::Vector3d* vec = new gz::msgs::Vector3d;

    // We cheat a little here and send the pose of the vehicle itself.
    vec->set_x(pose->Pos().X());
    vec->set_y(pose->Pos().Y());
    vec->set_z(pose->Pos().Z());
    resp.set_allocated_bearing(vec);

    std::ostringstream stream;
    resp.SerializeToOstream(&stream);
    message.set_data(stream.str());

    this->dataPtr->commsClient->SendPacket(message);
    this->dataPtr->messageQueue.pop();
  }
}

}

GZ_ADD_PLUGIN(tethys::RangeBearingPlugin,
  gz::sim::System,
  tethys::RangeBearingPlugin::ISystemConfigure,
  tethys::RangeBearingPlugin::ISystemPreUpdate)
