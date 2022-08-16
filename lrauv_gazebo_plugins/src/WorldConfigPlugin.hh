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

#include <gz/gui/Plugin.hh>

#include <gz/transport/Node.hh>

namespace tethys
{

/// \brief Control Panel for controlling the tethys using the GUI.
class WorldConfig : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief Constructor
  public: WorldConfig();

  /// \brief Destructor
  public: ~WorldConfig();

  /// \brief Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Set the file path to be loaded
  /// \param[in] _filePath The file path to be loaded
  public: Q_INVOKABLE void SetFilePath(QUrl _filePath);

  /// \brief Transport node
  private: gz::transport::Node node;

  /// \brief Transport publisher
  private: gz::transport::Node::Publisher pub;
};

}

#endif