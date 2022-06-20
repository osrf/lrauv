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

#include <gz/common/Console.hh>

#include <gz/plugin/Register.hh>

#include <gz/rendering/AxisVisual.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Text.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

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
    public: gz::rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the 3D scene
    public: gz::rendering::ScenePtr scene{nullptr};

    /// \brief ENU visual
    public: gz::rendering::VisualPtr enuVis{nullptr};

    /// \brief NED visual
    public: gz::rendering::VisualPtr nedVis{nullptr};

    /// \brief FSK visuals. The key is the model name, the value is the visual.
    public: std::map<std::string, gz::rendering::VisualPtr> fskVisuals;
  };
}

using namespace tethys;

/////////////////////////////////////////////////
ReferenceAxis::ReferenceAxis()
  : gz::gui::Plugin(),
    dataPtr(std::make_unique<ReferenceAxisPrivate>())
{
}

/////////////////////////////////////////////////
ReferenceAxis::~ReferenceAxis()
{
}

/////////////////////////////////////////////////
void ReferenceAxis::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Visualize point cloud";

  // Configuration
  if (_pluginElem)
  {
    for (auto fskElem = _pluginElem->FirstChildElement("fsk");
         fskElem != nullptr && fskElem->GetText() != nullptr;
         fskElem = fskElem->NextSiblingElement("fsk"))
    {
      this->dataPtr->fskVisuals[fskElem->GetText()] = nullptr;
    }
  }

  gz::gui::App()->findChild<
    gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool ReferenceAxis::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::PreRender::kType)
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

  this->scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!this->scene)
    return;

  for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
  {
    auto cam = std::dynamic_pointer_cast<gz::rendering::Camera>(
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
  // https://github.com/ignitionrobotics/ign-rendering/issues/500
  double hFOV = 2.0 * atan(tan(vFOV / 2.0) / aspectRatio);

  // TODO(chapulina) Let user choose distance from camera
  double xPos{2.0};

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
    this->enuVis->SetLocalScale(0.25, 0.25, 0.25);

    // Ogre2 doesn't support text yet
    // https://github.com/ignitionrobotics/ign-rendering/issues/487
    auto textGeom = this->scene->CreateText();
    if (nullptr != textGeom)
    {
      textGeom->SetFontName("Liberation Sans");
      textGeom->SetTextString("ENU");
      textGeom->SetShowOnTop(true);
      textGeom->SetCharHeight(0.1);
      textGeom->SetTextAlignment(gz::rendering::TextHorizontalAlign::RIGHT,
                                 gz::rendering::TextVerticalAlign::BOTTOM);

      auto textVis = this->scene->CreateVisual();
      textVis->AddGeometry(textGeom);
      textVis->SetLocalPosition(0.05, 0.05, 0.1);

      this->enuVis->AddChild(textVis);
    }
  }

  // TODO(chapulina) Let user choose Y offset
  double yOffset{1.0};

  // Set pose to be in front of camera
  double yPos = yOffset + (widthMeters - initialWidthMeters) * 0.5;
  auto enuPlacement = gz::math::Pose3d(xPos, yPos, 1.25, 0.0, 0.0, 0.0);
  auto enuPos = (this->camera->WorldPose() * enuPlacement).Pos();
  this->enuVis->SetLocalPosition(enuPos);

  // NED
  if (nullptr == this->nedVis)
  {
    this->nedVis = this->scene->CreateAxisVisual();
    this->scene->RootVisual()->AddChild(this->nedVis);
    this->nedVis->SetLocalRotation(GZ_PI, 0, GZ_PI * 0.5);
    this->nedVis->SetLocalScale(0.25, 0.25, 0.25);

    // Ogre2 doesn't support text yet
    auto textGeom = this->scene->CreateText();
    if (nullptr != textGeom)
    {
      textGeom->SetFontName("Liberation Sans");
      textGeom->SetTextString("NED");
      textGeom->SetShowOnTop(true);
      textGeom->SetCharHeight(0.1);
      textGeom->SetTextAlignment(gz::rendering::TextHorizontalAlign::LEFT,
                                 gz::rendering::TextVerticalAlign::TOP);

      auto textVis = this->scene->CreateVisual();
      textVis->AddGeometry(textGeom);
      textVis->SetLocalPosition(0.05, 0.05, 0.1);

      this->nedVis->AddChild(textVis);
    }
  }

  // Set pose to be in front of camera
  // TODO(chapulina) Let user choose Y offset
  yOffset = -1.0;
  yPos = yOffset + (initialWidthMeters - widthMeters) * 0.5;
  auto nedPlacement = gz::math::Pose3d(xPos, yPos, 1.35, 0.0, 0.0, 0.0);
  auto nedPos = (this->camera->WorldPose() * nedPlacement).Pos();
  this->nedVis->SetLocalPosition(nedPos);

  // FSK
  for (auto &[modelName, fskVis] : this->fskVisuals)
  {
    if (nullptr != fskVis)
      continue;

    auto vehicleVis = this->scene->VisualByName(modelName);
    if (nullptr == vehicleVis)
    {
      return;
    }

    fskVis = this->scene->CreateAxisVisual();
    vehicleVis->AddChild(fskVis);
    // TODO(chapulina) This rotation won't be needed if we update the model
    // https://github.com/osrf/lrauv/issues/80
    fskVis->SetLocalRotation(GZ_PI, 0, GZ_PI * 0.5);

    // Ogre2 doesn't support text yet
    auto textGeom = this->scene->CreateText();
    if (nullptr != textGeom)
    {
      textGeom->SetFontName("Liberation Sans");
      textGeom->SetTextString("FSK");
      textGeom->SetShowOnTop(true);
      textGeom->SetCharHeight(0.4);
      textGeom->SetTextAlignment(gz::rendering::TextHorizontalAlign::LEFT,
                                 gz::rendering::TextVerticalAlign::TOP);

      auto textVis = this->scene->CreateVisual();
      textVis->AddGeometry(textGeom);
      textVis->SetLocalPosition(0.2, 0.2, 0.5);

      fskVis->AddChild(textVis);
    }
  }
}

// Register this plugin
GZ_ADD_PLUGIN(tethys::ReferenceAxis,
                    gz::gui::Plugin)
