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

#include "WorldConfigPlugin.hh"
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

namespace tethys
{

WorldConfig::WorldConfig() : ignition::gui::Plugin()
{
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
    "WorldConfig", this);
  this->pub =
    this->node.Advertise<ignition::msgs::StringMsg>(
      "/world/science_sensor/environment_data_path");
}

WorldConfig::~WorldConfig()
{

}

void WorldConfig::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "World Configuration";

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

void WorldConfig::SetFilePath(QUrl _filePath)
{
  igndbg << "setting file path "<< _filePath.path().toStdString() <<"\n";

  ignition::msgs::StringMsg msg;
  msg.set_data(_filePath.path().toStdString());
  this->pub.Publish(msg);
}

}
// Register this plugin
IGNITION_ADD_PLUGIN(tethys::WorldConfig,
                    ignition::gui::Plugin)
