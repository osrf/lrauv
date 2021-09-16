#include "RangeBearingPlugin.hh"

#include "lrauv_range_bearing_request.pb.h"
#include "lrauv_range_bearing_response.pb.h"

#include <queue>

#include <ignition/gazebo/components.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>

#include <lrauv_ignition_plugins/comms/CommsClient.hh>

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
  public: void OnRecieveCommsMsg(
    const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage& message);

  /// \brief Callback for range requests
  public: void OnRangeRequest(
    const lrauv_ignition_plugins::msgs::LRAUVRangeBearingRequest& req);

  /// \brief Publish range-bearing response
  public: void PublishResponse(
    const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage& resp);

  /// \brief Queue which we need to respond to.
  public: std::queue<IncomingRangePing> messageQueue;

  /// \brief Table to lookup outgoing requests
  /// TODO(arjo) Add a garbage collection mechanism for packets that
  /// are lost.
  public: std::unordered_map<uint32_t,
    std::chrono::steady_clock::time_point> transmissionTime;

  /// \brief Transport node
  public: ignition::transport::Node node;

  /// \brief Publisher for results
  public: ignition::transport::Node::Publisher pub;

  /// \brief Processing duration
  public: std::chrono::steady_clock::duration processingDelay;

  /// \brief This object's address
  public: uint32_t address;

  /// \brief Current time
  public: std::chrono::steady_clock::time_point timeNow;

  /// \brief Link and entity which this is bound to
  public: ignition::gazebo::Entity linkEntity;

  /// \brief The current pose
  public: ignition::math::Pose3d currentPose;

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
    std::bind(&RangeBearingPrivateData::OnRecieveCommsMsg, this,
      std::placeholders::_1));
}

////////////////////////////////////////////////
void RangeBearingPrivateData::OnRecieveCommsMsg(
  const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage& message)
{
  using MsgType =
    lrauv_ignition_plugins::msgs::LRAUVAcousticMessage::MessageType;

  lrauv_ignition_plugins::msgs::LRAUVRangeBearingRequest req;
  std::istringstream stream{message.data()};
  req.ParseFromIstream(&stream);
  IncomingRangePing ping{message.from(), req.req_id(), this->timeNow};

  switch(message.type())
  {
  case MsgType::LRAUVAcousticMessage_MessageType_RangeRequest:  
    this->messageQueue.push(ping);
    break;
  case MsgType::LRAUVAcousticMessage_MessageType_RangeResponse:
    this->PublishResponse(message);
    break;
  default:
    ignwarn << "Unable to process message type\n";
  }
}

////////////////////////////////////////////////
void RangeBearingPrivateData::OnRangeRequest(
  const lrauv_ignition_plugins::msgs::LRAUVRangeBearingRequest& req)
{
  using Message = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;
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
  const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage& msg)
{
  lrauv_ignition_plugins::msgs::LRAUVRangeBearingResponse resp;
  std::istringstream stream{msg.data()};
  resp.ParseFromIstream(&stream);
  
  auto timeOfTx = transmissionTime[resp.req_id()];
  transmissionTime.erase(resp.req_id());
  auto duration = std::chrono::duration<double>(
    this->timeNow - timeOfTx - this->processingDelay);
  auto range = (this->speedOfSound * duration.count()) / 2;

  auto poseOffset = ignition::math::Matrix4d(this->currentPose);
  auto otherVehiclesPos =
    ignition::math::Vector3d(
      resp.bearing().x(), resp.bearing().y(), resp.bearing().z());
  auto groundTruthPose = poseOffset.Inverse() * otherVehiclesPos;

  auto theta = acos(groundTruthPose.Z() / groundTruthPose.Length());
  auto phi = atan2(groundTruthPose.Y(), groundTruthPose.X());

  lrauv_ignition_plugins::msgs::LRAUVRangeBearingResponse finalAnswer;
  finalAnswer.set_range(range);
  finalAnswer.set_req_id(resp.req_id());
  
  ignition::msgs::Vector3d* vec = new ignition::msgs::Vector3d;
  vec->set_x(groundTruthPose.Length()); vec->set_y(theta); vec->set_z(phi);
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
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  if (!_sdf->HasElement("address"))
  {
    ignerr << "<address> tag not found" << std::endl;
    return;
  }
  this->dataPtr->BindToAddress(_sdf->Get<uint32_t>("address"));

  if (!_sdf->HasElement("processing_delay"))
  {
    ignerr << "<processing_delay> not specified." << std::endl;
    return;
  }
  this->dataPtr->processingDelay = 
    std::chrono::steady_clock::duration{
      (long)(_sdf->Get<double>("processing_delay") * 1e9)};

  if (!_sdf->HasElement("speed_of_sound"))
  {
    ignerr << "<speed_of_sound> not specified\n";
    return;
  }
  this->dataPtr->speedOfSound = _sdf->Get<double>("speed_of_sound");

  if (!_sdf->HasElement("link_name"))
  {
    ignerr << 
      "<link_name> - expected the link name of the receptor" << std::endl;
    return;
  }
  auto vehicleModel = ignition::gazebo::Model(_entity);
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = vehicleModel.LinkByName(_ecm, linkName);
  if(this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
  {
    ignerr << "Link " << linkName << " was not found in "
      << vehicleModel.Name(_ecm) << std::endl;
    return;
  }
  ignition::gazebo::enableComponent<ignition::gazebo::components::WorldPose>(
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
    lrauv_ignition_plugins::msgs::LRAUVRangeBearingResponse>(
      this->dataPtr->topicPrefix + "responses");
}

////////////////////////////////////////////////
void RangeBearingPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  using MsgType =
    lrauv_ignition_plugins::msgs::LRAUVAcousticMessage::MessageType;

  if(_info.paused)
    return;

  ignition::gazebo::Link baseLink(this->dataPtr->linkEntity);
  auto pose = baseLink.WorldPose(_ecm);
  
  if (!pose.has_value())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
  this->dataPtr->currentPose = pose.value();
  this->dataPtr->timeNow = std::chrono::steady_clock::time_point{
    _info.simTime};
  
  while(
    !this->dataPtr->messageQueue.empty()
    && (this->dataPtr->messageQueue.front().timeOfReception 
      + this->dataPtr->processingDelay) <= this->dataPtr->timeNow)
  {
    auto ping = this->dataPtr->messageQueue.front();
    lrauv_ignition_plugins::msgs::LRAUVAcousticMessage message;
    lrauv_ignition_plugins::msgs::LRAUVRangeBearingResponse resp;
    message.set_to(ping.from);
    message.set_from(this->dataPtr->address);
    message.set_type(MsgType::LRAUVAcousticMessage_MessageType_RangeResponse);

    resp.set_req_id(ping.reqId);
    ignition::msgs::Vector3d* vec = new ignition::msgs::Vector3d;
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

IGNITION_ADD_PLUGIN(tethys::RangeBearingPlugin,
  ignition::gazebo::System,
  tethys::RangeBearingPlugin::ISystemConfigure,
  tethys::RangeBearingPlugin::ISystemPreUpdate)
