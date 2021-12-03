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

#include <ignition/rendering/AxisVisual.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Text.hh>
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
    public: void OnPreRender();

    /// \brief Initialize rendering
    public: void Initialize();

    /// \brief Pointer to the camera being recorded
    public: ignition::rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the 3D scene
    public: ignition::rendering::ScenePtr scene{nullptr};

    /// \brief ENU visual
    public: ignition::rendering::VisualPtr enuVis{nullptr};

    /// \brief NED visual
    public: ignition::rendering::VisualPtr nedVis{nullptr};

    /// \brief FSK visual
    public: ignition::rendering::VisualPtr fskVis{nullptr};
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
  if (_event->type() == ignition::gui::events::PreRender::kType)
  {
    this->dataPtr->OnPreRender();
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
void ReferenceAxisPrivate::OnPreRender()
{
  this->Initialize();

  if (!this->camera)
    return;

  // Assume the vertical FOV never changes, but the horizontal FOV does.
  // The API gives us HFOV only, so we calculate VFOV and store it.
  double aspectRatio = static_cast<double>(this->camera->ImageHeight()) /
      this->camera->ImageWidth();
  static double vFOV = -1;
  if (vFOV < 0)
  {
    double initialHFOV = this->camera->HFOV().Radian();
    vFOV = 2.0 * atan(tan(initialHFOV / 2.0) * aspectRatio);
  }

  // Calculate current HFOV because the API is always giving the initial value.
  double hFOV = 2.0 * atan(tan(vFOV / 2.0) / aspectRatio);

  // TODO(chapulina) Let user choose distance from camera
  double xPos{10.0};

  // Viewable width in meters at the desired distance
  double widthMeters = xPos * tan(hFOV*0.5);
  static double initialWidthMeters{-1};
  if (initialWidthMeters < 0)
  {
    initialWidthMeters = widthMeters;
  }

  // ENU
  if (nullptr == this->enuVis)
  {
    this->enuVis = this->scene->CreateAxisVisual();
    this->scene->RootVisual()->AddChild(this->enuVis);

    // Ogre2 doesn't support text yet
    // https://github.com/ignitionrobotics/ign-rendering/issues/487
    auto textGeom = this->scene->CreateText();
    if (nullptr != textGeom)
    {
      textGeom->SetFontName("Liberation Sans");
      textGeom->SetTextString("ENU");
      textGeom->SetShowOnTop(true);
      textGeom->SetCharHeight(0.4);
      textGeom->SetTextAlignment(ignition::rendering::TextHorizontalAlign::RIGHT,
                                 ignition::rendering::TextVerticalAlign::BOTTOM);

      auto textVis = this->scene->CreateVisual();
      textVis->AddGeometry(textGeom);
      textVis->SetLocalPosition(0.2, 0.2, 0.5);

      this->enuVis->AddChild(textVis);
    }
  }

  // TODO(chapulina) Let user choose Y offset
  double yOffset{5};

  // Set pose to be in front of camera
  double yPos = yOffset + (widthMeters - initialWidthMeters) * 0.5;
  auto enuPlacement = ignition::math::Pose3d(xPos, yPos, 6.0, 0.0, 0.0, 0.0);
  auto enuPos = (this->camera->WorldPose() * enuPlacement).Pos();
  this->enuVis->SetLocalPosition(enuPos);

  // NED
  if (nullptr == this->nedVis)
  {
    this->nedVis = this->scene->CreateAxisVisual();
    this->scene->RootVisual()->AddChild(this->nedVis);
    this->nedVis->SetLocalRotation(IGN_PI, 0, IGN_PI * 0.5);

    // Ogre2 doesn't support text yet
    auto textGeom = this->scene->CreateText();
    if (nullptr != textGeom)
    {
      textGeom->SetFontName("Liberation Sans");
      textGeom->SetTextString("NED");
      textGeom->SetShowOnTop(true);
      textGeom->SetCharHeight(0.4);
      textGeom->SetTextAlignment(ignition::rendering::TextHorizontalAlign::LEFT,
                                 ignition::rendering::TextVerticalAlign::TOP);

      auto textVis = this->scene->CreateVisual();
      textVis->AddGeometry(textGeom);
      textVis->SetLocalPosition(0.2, 0.2, 0.5);

      this->nedVis->AddChild(textVis);
    }
  }

  // Set pose to be in front of camera
  // TODO(chapulina) Let user choose Y offset
  yOffset = -5;
  yPos = yOffset + (initialWidthMeters - widthMeters) * 0.5;
  auto nedPlacement = ignition::math::Pose3d(xPos, yPos, 7.0, 0.0, 0.0, 0.0);
  auto nedPos = (this->camera->WorldPose() * nedPlacement).Pos();
  this->nedVis->SetLocalPosition(nedPos);

  // FSK
  if (nullptr == this->fskVis)
  {
    // TODO(chapulina) Support other names
    auto vehicleVis = this->scene->VisualByName("tethys");
    if (nullptr == vehicleVis)
    {
      return;
    }

    this->fskVis = this->scene->CreateAxisVisual();
    vehicleVis->AddChild(this->fskVis);
    // TODO(chapulina) This rotation won't be needed if we update the model
    // https://github.com/osrf/lrauv/issues/80
    this->fskVis->SetLocalRotation(0, IGN_PI, 0);

    // Ogre2 doesn't support text yet
    auto textGeom = this->scene->CreateText();
    if (nullptr != textGeom)
    {
      textGeom->SetFontName("Liberation Sans");
      textGeom->SetTextString("FSK");
      textGeom->SetShowOnTop(true);
      textGeom->SetCharHeight(0.4);
      textGeom->SetTextAlignment(ignition::rendering::TextHorizontalAlign::LEFT,
                                 ignition::rendering::TextVerticalAlign::TOP);

      auto textVis = this->scene->CreateVisual();
      textVis->AddGeometry(textGeom);
      textVis->SetLocalPosition(0.2, 0.2, 0.5);

      this->fskVis->AddChild(textVis);
    }
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(tethys::ReferenceAxis,
                    ignition::gui::Plugin)
