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

#ifndef TETHYS_RANGEBEARINGPLUGIN_HH_
#define TETHYS_RANGEBEARINGPLUGIN_HH_

#include <gz/sim/System.hh>

namespace tethys
{

class RangeBearingPrivateData;

///////////////////////////////////
/// \brief Range Bearing Plugin
/// This plugin simulates a two way ranging protocol over an acoustic network
/// including the delays in the system.
///
/// ## Parameters
/// * `<address>` - Address of the comms node you bind to.
/// * `<namespace>` - The namespace on which this plugin should listen to
///   receive and respond to requests.
/// * `<processing_delay>` - The amount of time which it will take for the
///   transponder to respond.
/// * `<speed_of_sound>` - Speed of sound in the underlying medium.
/// * `<link_name>` - The name of the link which holds the receiver.
///
/// ## External API for invoking the range bearing plugin.
///
/// The RangeBearingPlugin will listen on the 
/// `/{namespace}/range_bearing/requests` for incoming requests using the 
/// `lrauv_ignition_plugins::msgs::LRAUVRangeBearingRequest` message and will 
/// respond using the `lrauv_ignition_plugins::msgs::LRAUVRangeBearingResponse` 
/// message on the `/{namespace}/range_bearing/response` topic.
///
class RangeBearingPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: RangeBearingPlugin();

  /// Inherits documentation from parent class
  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/
  );

  /// Inherits documentation from parent class
  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm);

  /// Inherits documentation from parent class
  private: std::unique_ptr<RangeBearingPrivateData> dataPtr;
};
}

#endif
