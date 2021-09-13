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

#ifndef TETHYS_ACOUSTICCOMMS_ENGINE_
#define TETHYS_ACOUSTICCOMMS_ENGINE_

#include <ignition/gazebo/System.hh>

namespace tethys
{

class AcousticCommsPrivateData;

///////////////////////////////////
/// \brief 
/// * `<address>` : The address parameter. This is a 32 bit unsigned int which
/// reflects the address which belongs to this vehicle. This will also create 
/// the topics  `/comms/external/<address>/rx` on which this plugin will publish
/// received messages and `/comms/external/<address>/tx` which this plugin 
/// subscribes to and publishes data.
/// * `<model_plugin_file>` : This is the name of library containing the 
/// environmental comms Plugin. The file should be in your `$TETHYS_COMMS_MODEL`
/// directory [Required].
/// * `<model_name>` : This is the name of the environmental communications 
/// model. [Required]
/// * `<link_name>` : The link to which the wireless transponder is attached
/// to. [Required]
/// * `<external_comms_prefix>` : Prefix on which to handle interaction between
/// the application . [Optional, Defaults to `/comms/external`]
/// * `<internal_comms_prefix>` : Prefix on which to send and listen for
/// internal packets.  [Optional, Defaults to `/comms/internal`]
/// * `<broadcast>` : This defaults to true. When enabled all receivers will
/// receive all packets. If false, then packets will be directed only to 
/// specified receivers.  [Optional, Defaults to `/comms/internal`]
/// 
/// ```
///  <plugin
///         filename="AcousticCommsPlugin"
///         name="tethys::AcousticCommsPlugin">
///         <address>2</address>
///         <model_plugin_file>simpleacousticmodel</model_plugin_file>
///         <model_name>tethys::SimpleAcousticModel</model_name>
///         <link_name>base_link</link_name>
///        ...
///  </plugin>
/// ```
/// All other parameters are forwarded to the specific communication model.
class AcousticCommsPlugin:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
  public: AcousticCommsPlugin();

  /// Inherits documentation from parent class
  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/
  );

  /// Inherits documentation from parent class
  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm);

  /// Inherits documentation from parent class
  private: std::unique_ptr<AcousticCommsPrivateData> dataPtr;
};
}

#endif
