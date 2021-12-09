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

#ifndef TETHYS_REFERENCEAXIS_HH_
#define TETHYS_REFERENCEAXIS_HH_

#include <memory>

#include "ignition/msgs/pointcloud_packed.pb.h"

#include <ignition/gui/Plugin.hh>

namespace tethys
{
  class ReferenceAxisPrivate;

  /// \brief This plugin adds a few reference frames to the 3D scene:
  ///
  /// * ENU: floating op the user camera's top-left corner
  /// * NED: floating op the user camera's top-right corner
  /// * FSK: attached to all models specified in `<fsk>`
  ///
  /// For each frame, red-green-blue axes are spawned. When using Ogre 1, a
  /// floating text with the frame name is also spawned.
  class ReferenceAxis : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: ReferenceAxis();

    /// \brief Destructor
    public: ~ReferenceAxis() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<ReferenceAxisPrivate> dataPtr;
  };
}
#endif
