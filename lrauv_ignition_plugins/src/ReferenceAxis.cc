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

#include "ReferenceAxis.hh"

#include <ignition/common/Console.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

namespace tethys
{
  /// \brief Private data class for ReferenceAxis
  class ReferenceAxisPrivate
  {
    /// \brief Perform rendering actions in the render thread
    public: void OnRender();

    /// \brief Initialize rendering
    public: void Initialize();

    /// \brief Pointer to the camera being recorded
    public: ignition::rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the 3D scene
    public: ignition::rendering::ScenePtr scene{nullptr};
  };
}

using namespace tethys;

/////////////////////////////////////////////////
ReferenceAxis::ReferenceAxis()
  : ignition::gui::Plugin(),
    dataPtr(std::make_unique<ReferenceAxisPrivate>())
{
}

/////////////////////////////////////////////////
ReferenceAxis::~ReferenceAxis()
{
}

/////////////////////////////////////////////////
void ReferenceAxis::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualize point cloud";

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool ReferenceAxis::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void ReferenceAxisPrivate::Initialize()
{
  // Already initialized
  if (this->scene)
    return;

  this->scene = ignition::rendering::sceneFromFirstRenderEngine();
  if (!this->scene)
    return;

  for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
  {
    auto cam = std::dynamic_pointer_cast<ignition::rendering::Camera>(
      this->scene->NodeByIndex(i));
    if (cam && cam->HasUserData("user-camera") &&
        std::get<bool>(cam->UserData("user-camera")))
    {
      this->camera = cam;
      igndbg << "Video Recorder plugin is recoding camera ["
             << this->camera->Name() << "]" << std::endl;
      break;
    }
  }

  if (!this->camera)
  {
    ignerr << "Camera is not available" << std::endl;
  }
}

/////////////////////////////////////////////////
void ReferenceAxisPrivate::OnRender()
{
  this->Initialize();

  // TODO(chapulina)
  // Create axis visual
  // Add movable text according to axis being displayed
  // Set pose to be in front of camera (option to fix to world)
  // Set orientation to match requested world orientation (ENU, NED, etc)
}

// Register this plugin
IGNITION_ADD_PLUGIN(tethys::ReferenceAxis,
                    ignition::gui::Plugin)
