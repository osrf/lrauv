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

#include "SpawnPanelPlugin.hh"
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>

namespace tethys
{

SpawnPanel::SpawnPanel()
{
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
    "SpawnPanel", this);

  this->pub = this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVInit>(
      "/lrauv/init");
}

SpawnPanel::~SpawnPanel()
{

}

void SpawnPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Spawn LRAUV Panel";

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

void SpawnPanel::Spawn(
  double lattitude, double longitude, double depth, int commsId, QString name)
{
  ignition::msgs::StringMsg* strMsg = new ignition::msgs::StringMsg;
  strMsg->set_data(name.toStdString());

  lrauv_ignition_plugins::msgs::LRAUVInit msg;
  msg.set_initlat_(lattitude);
  msg.set_initlon_(longitude);
  msg.set_initz_(depth);
  msg.set_acommsaddress_(commsId);
  msg.set_allocated_id_(strMsg);
  this->pub.Publish(msg);
}

void SpawnPanel::Update(const ignition::gazebo::UpdateInfo &,
  ignition::gazebo::EntityComponentManager &_ecm)
{
}

}

// Register this plugin
IGNITION_ADD_PLUGIN(tethys::SpawnPanel,
                    ignition::gui::Plugin)
