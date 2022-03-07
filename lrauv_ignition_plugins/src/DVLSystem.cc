#include "dvl/DVLSensor.hh"
#include "DVLSystem.hh"

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/CustomSensor.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Name.hh>

#include <ignition/gazebo/Util.hh>

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
  _ecm.EachNew<ignition::gazebo::components::CustomSensor,
               ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *_custom,
        const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        // Get sensor's scoped name without the world
        auto sensorScopedName = ignition::gazebo::removeParentScope(
            ignition::gazebo::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/pathfinder";
          data.SetTopic(topic);
        }

        ignition::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<DVLSensor>(data);
        if (nullptr == sensor)
        {
          ignerr << "Failed to create odometer [" << sensorScopedName << "]"
                 << std::endl;
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<ignition::gazebo::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            ignition::gazebo::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        return true;
      });
}

void DVLSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }

  
}

IGNITION_ADD_PLUGIN(
  tethys::DVLSystem,
  ignition::gazebo::System,
  tethys::DVLSystem::ISystemConfigure,
  tethys::DVLSystem::ISystemPreUpdate,
  tethys::DVLSystem::ISystemPostUpdate)