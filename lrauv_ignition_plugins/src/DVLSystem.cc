#include "dvl/DVLSensor.hh"
#include "DVLSystem.hh"

#include <ignition/plugin/Register.hh>

using namespace tethys;

DVLSystem::DVLSystem()
{
}

void DVLSystem::Configure(const ignition::gazebo::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        ignition::gazebo::EntityComponentManager &_ecm,
                        ignition::gazebo::EventManager &_eventMgr)
{

}

void DVLSystem::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                        ignition::gazebo::EntityComponentManager &_ecm)
{

}

IGNITION_ADD_PLUGIN(
  tethys::DVLSystem,
  ignition::gazebo::System,
  tethys::DVLSystem::ISystemConfigure,
  tethys::DVLSystem::ISystemPreUpdate)