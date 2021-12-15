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

/// \brief Control Panel for controlling the tethys using the GUI.
class ControlPanel : public ignition::gui::Plugin
{
  Q_OBJECT

  /// \brief Constructor
  public: ControlPanel();

  /// \brief Destructor
  public: ~ControlPanel();

  /// \brief Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Releases the drop weight
  public: Q_INVOKABLE void ReleaseDropWeight();

  /// \brief Sets the vehicle name that you want to control
  /// \param[in] _name - The name of the vehicle in question.
  public: Q_INVOKABLE void SetVehicle(QString _name);

  /// \brief Sets the rudder rotation
  /// \param[in] _rudderAngle - The rudder angle set point
  public: Q_INVOKABLE void SetRudder(qreal _rudderAngle);

  /// \brief Sets the elevator rotation
  /// \param[in] _elevatorAngle - The elevator angle set point
  public: Q_INVOKABLE void SetElevator(qreal _elevatorAngle);

  /// \brief Sets the mass shifter position
  /// \param[in] _pitchmassPosition - The mass shifter position
  public: Q_INVOKABLE void SetPitchMass(qreal _pitchmassPosition);

  /// \brief Sets the thruster thrust
  /// \param[in] _rudderAngle - The thruster set point
  public: Q_INVOKABLE void SetThruster(qreal _thrust);

  /// \brief Sets the buoyancy engine
  /// \param[in] _volume- The thruster set point
  public: Q_INVOKABLE void SetBuoyancyEngine(qreal _volume);

  /// \brief Transport node
  private: ignition::transport::Node node;

  /// \brief Transport publisher
  private: ignition::transport::Node::Publisher pub;

  /// \brief LRAUVCommand for the last state
  private: lrauv_ignition_plugins::msgs::LRAUVCommand lastCommand;

  /// \brief The vehicle name to subscribe to.
  private: std::string vehicleName;
};

}

#endif
