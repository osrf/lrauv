#include "RangeBearingPlugin.hh"

namespace tethys
{
////////////////////////////////////////////////
class RangeBearingPrivateData
{

};

////////////////////////////////////////////////
RangeBearingPlugin::RangeBearingPlugin():
  dataPtr(std::make_unique<RangeBearingPrivateData>())
{
}

////////////////////////////////////////////////
void RangeBearingPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
}

////////////////////////////////////////////////
void RangeBearingPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
}

}