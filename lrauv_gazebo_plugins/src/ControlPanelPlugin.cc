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
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

namespace tethys
{

ControlPanel::ControlPanel() : gz::gui::Plugin()
{
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
    "ControlPanel", this);

  // Start neutral - these values should match the defaults on the QML
  lastCommand.set_buoyancyaction_(500.0 / (100 * 100 * 100));
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

  gz::gui::App()->findChild<
    gz::gui::MainWindow *>()->installEventFilter(this);
}

void ControlPanel::ReleaseDropWeight()
{
  gzdbg << "release dropweight\n";
  lastCommand.set_dropweightstate_(0);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetVehicle(QString _name)
{
  gzdbg << "Setting name as " << _name.toStdString() <<"\n";
  this->pub = node.Advertise<lrauv_gazebo_plugins::msgs::LRAUVCommand>(
    "/" + _name.toStdString() + "/command_topic"
  );
}

void ControlPanel::SetRudder(qreal _angle)
{
  gzdbg << "Setting rudder angle to " << _angle << "\n";
  lastCommand.set_rudderangleaction_(_angle);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetElevator(qreal _angle)
{
  gzdbg << "Setting elevator angle to " << _angle << "\n";
  lastCommand.set_elevatorangleaction_(_angle);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetPitchMass(qreal _massPosition)
{
  gzdbg << "Setting mass position angle to " << _massPosition << "\n";
  lastCommand.set_masspositionaction_(_massPosition);
  this->pub.Publish(lastCommand);
}


void ControlPanel::SetThruster(qreal _thrust)
{
  gzdbg << "Setting thruster angular velocity to " << _thrust << "\n";
  lastCommand.set_propomegaaction_(_thrust);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetBuoyancyEngine(qreal _volume)
{
  gzdbg << "Setting buoyancy engine to " << _volume << "\n";
  lastCommand.set_buoyancyaction_(_volume);
  this->pub.Publish(lastCommand);
}
}

// Register this plugin
GZ_ADD_PLUGIN(tethys::ControlPanel,
                    gz::gui::Plugin)
