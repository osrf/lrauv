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

#ifndef TETHYS_CONTROLPANEL_HH_
#define TETHYS_CONTROLPANEL_HH_

#include <ignition/gui/Plugin.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gazebo/gui/GuiSystem.hh>

#include "lrauv_ignition_plugins/lrauv_init.pb.h"

namespace tethys
{

/// \brief Control Panel for controlling the tethys using the GUI.
class SpawnPanel : public ignition::gazebo::GuiSystem
{
  Q_OBJECT

  /// \brief Constructor
  public: SpawnPanel();

  /// \brief Destructor
  public: ~SpawnPanel();

  /// \brief Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Releases the drop weight
  public: Q_INVOKABLE void Spawn(
    double lattitude, double longitude, double depth, int commsId, QString name);

  /// \brief Documentation inherited
  public: void Update(const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Transport node
  private: ignition::transport::Node node;

  /// \brief Transport publisher
  private: ignition::transport::Node::Publisher pub;

  /// \brief The names of all the models
  private: std::unordered_set<std::string> modelNames;

  /// \brief The acoustic address of the models
  private: std::unordered_set<int> acousticIds;
};

}

#endif
