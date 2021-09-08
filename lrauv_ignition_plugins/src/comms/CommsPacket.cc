#include <lrauv_ignition_plugins/comms/CommsPacket.hh>

using namespace tethys;

//////////////////////////////////////////////////
class tethys::CommsPacket::CommsPacketPrivateData
{
public:
  ignition::math::Vector3d position;
  uint32_t to;
  uint32_t from;
  uint32_t type;
  std::chrono::steady_clock::time_point timeOfTx;
  std::string data;
};

//////////////////////////////////////////////////
CommsPacket::CommsPacket() : dataPtr(std::make_shared<CommsPacketPrivateData>())
{
  // do nothing
}

//////////////////////////////////////////////////
CommsPacket::~CommsPacket()
{
  // do nothing
}

//////////////////////////////////////////////////
ignition::math::Vector3d CommsPacket::Position() const
{
  return this->dataPtr->position;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::time_point CommsPacket::TimeOfTransmission() const
{
  return this->dataPtr->timeOfTx;
}

//////////////////////////////////////////////////
uint32_t CommsPacket::To() const
{
  return this->dataPtr->to;
}

//////////////////////////////////////////////////
uint32_t CommsPacket::From() const
{
  return this->dataPtr->from;
}

//////////////////////////////////////////////////
std::string CommsPacket::Data() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
lrauv_ignition_plugins::msgs::LRAUVAcousticMessage 
  CommsPacket::ToExternalMsg() const
{
  lrauv_ignition_plugins::msgs::LRAUVAcousticMessage msg;
  msg.set_data(this->dataPtr->data);
  msg.set_from(this->dataPtr->from);
  msg.set_to(this->dataPtr->to);
  //msg.set_type();
  return msg;
}

//////////////////////////////////////////////////
lrauv_ignition_plugins::msgs::LRAUVInternalComms
  CommsPacket::ToInternalMsg() const
{
  lrauv_ignition_plugins::msgs::LRAUVInternalComms msg;
  msg.set_data(this->dataPtr->data);
  msg.set_from(this->dataPtr->from);
  msg.set_to(this->dataPtr->to);

  ignition::msgs::Vector3d vector;
  vector.set_x(this->dataPtr->position.X());
  vector.set_y(this->dataPtr->position.Y());
  vector.set_z(this->dataPtr->position.Z());
  msg.set_allocated_position(&vector);

  ignition::msgs::Time time;
  auto sec = 
    std::chrono::duration_cast<std::chrono::seconds>(
      this->dataPtr->timeOfTx.time_since_epoch()
    );
  time.set_sec(sec.count());
  time.set_nsec(
    this->dataPtr->timeOfTx.time_since_epoch().count()
    - (long)(sec.count() * 1e9));

  ignition::msgs::Header header;
  header.set_allocated_stamp(&time);

  msg.set_allocated_header(&header);
  //msg.set_type();
  return msg;
}

//////////////////////////////////////////////////
CommsPacket CommsPacket::make(
  const lrauv_ignition_plugins::msgs::LRAUVAcousticMessage& datapayload,
  const ignition::math::Vector3d position,
  const std::chrono::steady_clock::time_point timeOfTx)
{
  CommsPacket packet;
  packet.dataPtr->position = position;
  packet.dataPtr->to = datapayload.to();
  packet.dataPtr->from = datapayload.from();
  packet.dataPtr->data = datapayload.data();
  packet.dataPtr->position = position;
  packet.dataPtr->timeOfTx = timeOfTx;
  return packet;
}

//////////////////////////////////////////////////
CommsPacket CommsPacket::make(
  const lrauv_ignition_plugins::msgs::LRAUVInternalComms& datapayload)
{
  CommsPacket packet;
  packet.dataPtr->to = datapayload.to();
  packet.dataPtr->from = datapayload.from();
  packet.dataPtr->data = datapayload.data();
  
  ignition::math::Vector3d position;
  position.X(datapayload.position().x());
  position.Y(datapayload.position().y());
  position.Z(datapayload.position().z());
  packet.dataPtr->position = position;

  auto timeMsg = datapayload.header().stamp();
  std::chrono::nanoseconds dur((long)(timeMsg.sec() * 1e9) + timeMsg.nsec());
  std::chrono::steady_clock::time_point timeOfTx(dur);
  packet.dataPtr->timeOfTx = timeOfTx;
  return packet;
}
