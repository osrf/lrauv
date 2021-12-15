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

#include "lrauv_command.pb.h"

namespace tethys
{

class ControlPanel : public ignition::gui::Plugin
{
  Q_OBJECT

  public: ControlPanel();

  public: ~ControlPanel();

  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  public: Q_INVOKABLE void ReleaseDropWeight();

  public: Q_INVOKABLE void SetVehicle(QString _name);

  public: Q_INVOKABLE void SetRudder(qreal _rudderAngle);

  public: Q_INVOKABLE void SetElevator(qreal _elevatorAngle);

  public: Q_INVOKABLE void SetPitchMass(qreal _pitchmassAngle);

  public: Q_INVOKABLE void SetThruster(qreal _pitchmassAngle);

  private: ignition::transport::Node node;

  private: ignition::transport::Node::Publisher pub;

  private: lrauv_ignition_plugins::msgs::LRAUVCommand lastCommand;

  private: std::string vehicleName;
};

}

#endif