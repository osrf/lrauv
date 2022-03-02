#include "dvl/DVLSensor.hh"
#include "DVLSystem.hh"

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
