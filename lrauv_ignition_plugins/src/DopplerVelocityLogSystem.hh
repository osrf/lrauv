#ifndef DOPPLERVELOCITYLOGSYSTEM_H_
#define DOPPLERVELOCITYLOGSYSTEM_H_

#include <gz/sim/System.hh>

namespace tethys
{

class DopplerVelocityLogSystemPrivate;

/// \brief System that creates and updates DopplerVelocityLog (DVL) sensors.
class DopplerVelocityLogSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: DopplerVelocityLogSystem();

  public: ~DopplerVelocityLogSystem();

  /// Inherits documentation from parent class
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr
  ) override;

  /// Inherits documentation from parent class
  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  /// Inherits documentation from parent class
  public: void Update(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  /// Inherits documentation from parent class
  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<DopplerVelocityLogSystemPrivate> dataPtr;
};

}  // namespace tethys

#endif // DOPPLERVELOCITYLOGSYSTEM_H_
