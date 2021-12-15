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

#include "ControlPanelPlugin.hh"
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

namespace tethys
{

ControlPanel::ControlPanel() : ignition::gui::Plugin()
{
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
    "ControlPanel", this);

  lastCommand.set_buoyancyaction_(0.0005);
  lastCommand.set_dropweightstate_(1);
  this->SetVehicle("tethys");
}

ControlPanel::~ControlPanel()
{

}

void ControlPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Tethys Control Panel";

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

void ControlPanel::ReleaseDropWeight()
{
  igndbg << "release dropweight\n";
  lastCommand.set_dropweightstate_(0);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetVehicle(QString _name)
{
  igndbg << "Setting name as " << _name.toStdString() <<"\n";
  vehicleName = _name.toStdString();
  this->pub = node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(
    "/" + vehicleName + "/command_topic"
  );
}

void ControlPanel::SetRudder(qreal _angle)
{
  igndbg << "Setting rudder angle to " << _angle << "\n";
  lastCommand.set_rudderangleaction_(_angle);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetElevator(qreal _angle)
{
  igndbg << "Setting elevator angle to " << _angle << "\n";
  lastCommand.set_elevatorangleaction_(_angle);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetPitchMass(qreal _angle)
{
  igndbg << "Setting mass position angle to " << _angle << "\n";
  lastCommand.set_masspositionaction_(_angle);
  this->pub.Publish(lastCommand);
}
}

// Register this plugin
IGNITION_ADD_PLUGIN(tethys::ControlPanel,
                    ignition::gui::Plugin)
