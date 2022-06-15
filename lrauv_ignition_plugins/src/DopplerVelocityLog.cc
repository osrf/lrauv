/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <optional>
#include <unordered_map>
#include <vector>

// TODO(hidmic): implement SVD in ignition?
#include <Eigen/Core>
#include <Eigen/SVD>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RayQuery.hh>

#include <gz/sensors/Manager.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/RenderingEvents.hh>
#include <gz/sensors/RenderingSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/SensorTypes.hh>

#include <gz/sim/Entity.hh>

#include <gz/transport/Node.hh>

#include "lrauv_ignition_plugins/dvl_beam_state.pb.h"
#include "lrauv_ignition_plugins/dvl_kinematic_estimate.pb.h"
#include "lrauv_ignition_plugins/dvl_range_estimate.pb.h"
#include "lrauv_ignition_plugins/dvl_tracking_target.pb.h"
#include "lrauv_ignition_plugins/dvl_velocity_tracking.pb.h"

#include "DopplerVelocityLog.hh"

namespace tethys
{
namespace
{


/// \brief Axis-aligned patch on a plane, using image frame conventions.
template <typename T>
class AxisAlignedPatch2
{
  public: AxisAlignedPatch2() = default;

  public: AxisAlignedPatch2(
      const gz::math::Vector2<T> &_topLeft,
      const gz::math::Vector2<T> &_bottomRight)
    : topLeft(_topLeft), bottomRight(_bottomRight)
  {
  }

  /// \brief Scalar converting copy constructor
  public: template<typename U>
  AxisAlignedPatch2(const AxisAlignedPatch2<U> &_other)
  {
    this->topLeft.X(static_cast<T>(_other.XMax()));
    this->topLeft.Y(static_cast<T>(_other.YMax()));
    this->bottomRight.X(static_cast<T>(_other.XMin()));
    this->bottomRight.Y(static_cast<T>(_other.YMin()));
  }

  public: T XMax() const { return this->topLeft.X(); }

  public: T XMin() const { return this->bottomRight.X(); }

  public: T XSize() const { return this->XMax() - this->XMin(); }

  public: T YMax() const { return this->topLeft.Y(); }

  public: T YMin() const { return this->bottomRight.Y(); }

  public: T YSize() const { return this->YMax() - this->YMin(); }

  /// \brief Merge patch with `_other`.
  /// \return a patch that includes both.
  public: AxisAlignedPatch2<T>& Merge(const AxisAlignedPatch2<T> &_other)
  {
    this->topLeft.Set(
        std::max(this->topLeft.X(), _other.topLeft.X()),
        std::max(this->topLeft.Y(), _other.topLeft.Y()));
    this->bottomRight.Set(
        std::min(this->bottomRight.X(), _other.bottomRight.X()),
        std::min(this->bottomRight.Y(), _other.bottomRight.Y()));
    return *this;
  }

  /// \brief Flip patch, sending each corner to the opposite quadrant.
  public: AxisAlignedPatch2<T> Flip() const
  {
    return {-this->bottomRight, -this->topLeft};
  }

  /// \brief Broadcast multiply corner coordinates by `_vector` coordinates.
  const AxisAlignedPatch2<T> operator*(gz::math::Vector2<T> _vector) const
  {
    return {this->topLeft * _vector, this->bottomRight * _vector};
  }

  /// \brief Broadcast divide corner coordinates by `_vector` coordinates.
  const AxisAlignedPatch2<T> operator/(gz::math::Vector2<T> _vector) const
  {
    return {this->topLeft / _vector, this->bottomRight / _vector};
  }

  /// \brief Broadcast sum corner coordinates with `_vector` coordinates.
  const AxisAlignedPatch2<T> operator+(gz::math::Vector2<T> _vector) const
  {
    return {this->topLeft + _vector, this->bottomRight + _vector};
  }

  /// \brief Broadcast substract corner coordinate with `_vector` coordinates.
  const AxisAlignedPatch2<T> operator-(gz::math::Vector2<T> _vector) const
  {
    return {this->topLeft - _vector, this->bottomRight - _vector};
  }

  /// \brief Upper-left corner i.e. (x, y) maxima
  private: gz::math::Vector2<T> topLeft;

  /// \brief Bottom-right corner i.e (x, y) minima
  private: gz::math::Vector2<T> bottomRight;
};

// Handy type definitions
using AxisAlignedPatch2d = AxisAlignedPatch2<double>;
using AxisAlignedPatch2i = AxisAlignedPatch2<int>;

/// \brief Acoustic DVL beam, modelled as a circular cone with aperture
/// angle α and its apex at the origin. Its axis of symmetry is nominally
/// aligned with the x-axis of an x-forward, y-left, z-up frame (following
/// usual Gazebo frame conventions, typically facing downwards).
///
///          +            The cone may be tilted w.r.t. the x-axis
///         /|\           and rotated about the same to accomodate
///        / | \          different beam arrangements, in that order.
///       /  |  \         That is, an extrinsic XY rotation applies.
///      /   v   \
///         x
///      |-------|
///          α
class AcousticBeam
{
  /// \brief Acoustic beam constructor.
  /// \param[in] _id ID of the beam. Ought to be unique.
  /// \param[in] _apertureAngle Aperture angle α of the beam.
  /// \param[in] _rotationAngle Rotation angle ψ of the beam
  /// i.e. a rotation about the x-axis of its frame.
  /// \param[in] _tiltAngle Tilt angle φ of the
  /// beam i.e. a rotation about the y-axis of its frame,
  /// away from the x-axis. Must lie in the (-90, 90) degrees
  /// interval.
  public: AcousticBeam(
      const int _id,
      const gz::math::Angle _apertureAngle,
      const gz::math::Angle _rotationAngle,
      const gz::math::Angle _tiltAngle)
    : id(_id), apertureAngle(_apertureAngle),
      normalizedRadius(std::atan(_apertureAngle.Radian() / 2.))
  {
    // Use extrinsic XY convention (as it is easier to reason about)
    using Quaterniond = gz::math::Quaterniond;
    this->transform.Rot() =
      Quaterniond::EulerToQuaternion(_rotationAngle.Radian(), 0., 0.) *
      Quaterniond::EulerToQuaternion(0., _tiltAngle.Radian(), 0.);
    this->axis = this->transform.Rot() * gz::math::Vector3d::UnitX;
    const gz::math::Angle azimuthAngle =
        std::atan2(this->axis.Y(), this->axis.X());
    const gz::math::Angle inclinationAngle =
        std::atan2(this->axis.Z(), std::sqrt(
            std::pow(this->axis.X(), 2.) +
            std::pow(this->axis.Y(), 2.)));
    const gz::math::Vector2d topLeft{
      (azimuthAngle + _apertureAngle / 2.).Radian(),
      (inclinationAngle + _apertureAngle / 2.).Radian()};
    const gz::math::Vector2d bottomRight{
      (azimuthAngle - _apertureAngle / 2.).Radian(),
      (inclinationAngle - _apertureAngle / 2.).Radian()};
    this->sphericalFootprint = AxisAlignedPatch2d{topLeft, bottomRight};
  }

  public: int Id() const { return this->id; }

  public: const gz::math::Pose3d &Transform() const
  {
    return this->transform;
  }

  public: const gz::math::Vector3d &Axis() const
  {
    return this->axis;
  }

  public: double NormalizedRadius() const
  {
    return this->normalizedRadius;
  }

  public: const gz::math::Angle &ApertureAngle() const
  {
    return this->apertureAngle;
  }

  public: const AxisAlignedPatch2d &SphericalFootprint() const
  {
    return this->sphericalFootprint;
  }

  private: int id;

  private: gz::math::Angle apertureAngle;

  private: double normalizedRadius;

  private: gz::math::Pose3d transform;

  private: gz::math::Vector3d axis;

  private: AxisAlignedPatch2d sphericalFootprint;
};

/// \brief DVL acoustic beam reflecting target
///
/// Pose is defined w.r.t. to the beams frame
struct Target
{
  gz::math::Pose3d pose;
  gz::sim::Entity entity;
};

}

using namespace lrauv_ignition_plugins::msgs;

/// \brief Private data for DopplerVelocityLog
class DopplerVelocityLogPrivate
{
  /// \brief SDF DOM object
  public: sdf::ElementPtr sensorSdf;

  public: using DVLType = DVLVelocityTracking_DVLType;

  /// \brief Dictionary of known DVL types
  public: const std::unordered_map<std::string, DVLType> knownDVLTypes{
    {"piston", DVLVelocityTracking::DVL_TYPE_PISTON},
    {"phased_array", DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY}};

  /// \brief Type of DVL
  public: DVLType dvlType;

  /// \brief Sensor entity ID (for world state lookup)
  public: gz::sim::Entity entityId{gz::sim::kNullEntity};

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief State of all entities in the world.
  public: WorldKinematicState worldState;

  /// \brief Depth sensor (i.e. a GPU raytracing sensor).
  public: gz::rendering::GpuRaysPtr depthSensor;

  /// \brief Image sensor (i.e. a camera sensor) to aid ray querys.
  public: gz::rendering::CameraPtr imageSensor;

  /// \brief Depth sensor resolution at 1 m distance, in meters.
  public: double resolution;

  /// \brief Depth sensor intrinsic constants
  public: struct {
    gz::math::Vector2d offset; ///<! Azimuth and elevation offsets
    gz::math::Vector2d step;  ///<! Azimuth and elevation steps
  } depthSensorIntrinsics;

  /// \brief Connection from depth camera with new depth data.
  public: gz::common::ConnectionPtr depthConnection;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief DVL acoustic beams' description
  public: std::vector<AcousticBeam> beams;

  /// \brief Transform from sensor frame to reference frame.
  ///
  /// Useful to cope with different DVL frame conventions.
  public: gz::math::Pose3d referenceFrameTransform;

  /// \brief Transform from sensor frame to acoustic beams' frame.
  ///
  /// I.e. x-forward, y-left, z-up (dvl sensor frame) rotates to
  /// x-down, y-left, z-forward (acoustic beams' frame).
  public: const gz::math::Pose3d beamsFrameTransform{
    gz::math::Vector3d::Zero,
    gz::math::Quaterniond{0., M_PI/2., 0.}};

  /// \brief DVL acoustic beams' targets
  public: std::vector<std::optional<Target>> beamTargets;

  /// \brief DVL acoustic beams' patches in depth scan frame
  public: std::vector<AxisAlignedPatch2i> beamScanPatches;

  /// \brief DVL noise model
  public: gz::sensors::NoisePtr noiseModel;

  /// \brief Node to create a topic publisher with
  public: gz::transport::Node node;

  /// \brief Publisher for velocity tracking messages
  public: gz::transport::Node::Publisher pub;

  /// \brief Flag to indicate if sensor is generating data
  public: bool generatingData = false;

  /// \brief DVL acoustic beams' lobe markers
  public: gz::msgs::Marker_V beamLobesMessage;

  /// \brief Whether to show beam visual aids
  public: bool visualizeBeamLobes = false;

    /// \brief DVL acoustic beams' reflection markers
  public: gz::msgs::Marker_V beamReflectionsMessage;

  /// \brief Whether to show beam visual aids
  public: bool visualizeBeamReflections = false;
};

//////////////////////////////////////////////////
DopplerVelocityLog::DopplerVelocityLog()
  : dataPtr(new DopplerVelocityLogPrivate())
{
}

//////////////////////////////////////////////////
DopplerVelocityLog::~DopplerVelocityLog()
{
}

//////////////////////////////////////////////////
bool DopplerVelocityLog::Load(const sdf::Sensor &_sdf)
{
  if (!gz::sensors::RenderingSensor::Load(_sdf))
  {
    return false;
  }

  // Check if this sensor is of the right type
  if (_sdf.Type() != sdf::SensorType::CUSTOM)
  {
    ignerr << "Expected [" << this->Name() << "] sensor to be "
           << "a DVL but found a " << _sdf.TypeStr() << "."
           << std::endl;
    return false;
  }

  sdf::ElementPtr elem = _sdf.Element();
  if (!elem->HasAttribute("ignition:type"))
  {
    ignerr << "Missing 'ignition:type' attribute "
           << "for sensor [" << this->Name() << "]. "
           << "Aborting load." << std::endl;
    return false;
  }
  const auto type = elem->Get<std::string>("ignition:type");
  if (type != "dvl")
  {
    ignerr << "Expected sensor [" << this->Name() << "] to be a "
           << "DVL but it is of '" << type << "' type. Aborting load."
           << std::endl;
    return false;
  }
  if (!elem->HasElement("ignition:dvl"))
  {
    ignerr << "Missing 'ignition:dvl' configuration for "
           << "sensor [" << this->Name() << "]. "
           << "Aborting load." << std::endl;
    return false;
  }
  this->dataPtr->sensorSdf = elem->GetElement("ignition:dvl");

  // Instantiate interfaces
  this->dataPtr->pub =
      this->dataPtr->node.Advertise<DVLVelocityTracking>(this->Topic());
  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic "
           << "[" << this->Topic() << "] for sensor "
           << "[" << this->Name() << "]" << std::endl;
    return false;
  }

  // Setup sensors
  if (this->Scene())
  {
    if (!this->CreateRenderingSensors())
    {
      ignerr << "Failed to create sensors for "
             << "[" << this->Name() << "] sensor. "
             << "Aborting load." << std::endl;
      return false;
    }
  }

  ignmsg << "Loaded sensor [" << this->Name() << "] DVL sensor." << std::endl;
  this->dataPtr->sceneChangeConnection =
      gz::sensors::RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&DopplerVelocityLog::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool DopplerVelocityLog::CreateRenderingSensors()
{
  ignmsg << "Initializing [" << this->Name() << "] sensor." << std::endl;
  const auto dvlTypeName =
      this->dataPtr->sensorSdf->Get<std::string>("type", "piston").first;
  if (this->dataPtr->knownDVLTypes.count(dvlTypeName) == 0)
  {
    ignerr << "[" << this->Name() << "] specifies an unknown"
           << " DVL type '" << dvlTypeName << "'" << std::endl;
    return false;
  }
  this->dataPtr->dvlType = this->dataPtr->knownDVLTypes.at(dvlTypeName);

  this->dataPtr->beams.clear();
  this->dataPtr->beamTargets.clear();

  int defaultBeamId = 0;
  sdf::ElementPtr arrangementElement =
      this->dataPtr->sensorSdf->GetElement("arrangement");
  if (!arrangementElement)
  {
    ignerr << "No beam arrangement specified for "
           << "[" << this->Name() << "] sensor"
           << std::endl;
    return false;
  }
  const bool useDegrees =
      arrangementElement->Get("degrees", false).first;
  const gz::math::Angle angleUnit = useDegrees ? IGN_DTOR(1.) : 1.;

  sdf::ElementPtr beamElement = arrangementElement->GetElement("beam");
  while (beamElement)
  {
    // Fetch acoustic beam specification
    const auto beamId =
        beamElement->Get<int>("id", defaultBeamId).first;
    const auto beamApertureAngle = (beamElement->Get<gz::math::Angle>(
        "aperture", gz::math::Angle::HalfPi).first * angleUnit).Normalized();
    const auto beamRotationAngle = (beamElement->Get<gz::math::Angle>(
        "rotation", gz::math::Angle::Zero).first * angleUnit).Normalized();
    const auto beamTiltAngle = (beamElement->Get<gz::math::Angle>(
        "tilt", gz::math::Angle::Zero).first * angleUnit).Normalized();
    if (std::abs(beamTiltAngle.Radian()) >=
        std::abs(gz::math::Angle::HalfPi.Radian()))
    {
      ignerr << "Invalid tilt angle for beam #" << beamId
             << " of [" << this->Name() << "]" << " sensor: "
             << beamTiltAngle.Radian() << " rads "
             << "(" << beamTiltAngle.Degree() << " degrees) "
             << "not in the (-90, 90) degree interval." << std::endl;
      return false;
    }

    // Build acoustic beam
    this->dataPtr->beams.push_back(AcousticBeam{
        beamId, beamApertureAngle, beamRotationAngle,
        beamTiltAngle});

    ignmsg << "Adding acoustic beam #" << beamId
           << " to [" << this->Name() << "] sensor. "
           << "Beam has a " << beamApertureAngle.Radian() << " rads "
           << "(" << beamApertureAngle.Degree() << " degrees) aperture angle, "
           << "it exhibits a " << beamTiltAngle.Radian() << " rads "
           << "(" << beamTiltAngle.Degree() << " degrees) tilt, "
           << "and it is rotated " << beamRotationAngle.Radian() << " rads "
           << "(" << beamRotationAngle.Degree() << " degrees)."
           << std::endl;

    defaultBeamId = this->dataPtr->beams.back().Id() + 1;
    beamElement = beamElement->GetNextElement("beam");
  }

  if (this->dataPtr->beams.size() < 3)
  {
    ignerr << "Expected at least three (3) beams "
           << "for [" << this->Name() << "] sensor."
           << std::endl;
    return false;
  }
  // Add as many (still null) targets as beams
  this->dataPtr->beamTargets.resize(this->dataPtr->beams.size());

  this->dataPtr->depthSensor =
      this->Scene()->CreateGpuRays(this->Name() + "_depth_sensor");
  if (!this->dataPtr->depthSensor)
  {
    ignerr << "Failed to create depth sensor for "
           << "for [" << this->Name() << "] sensor."
           << std::endl;
    return false;
  }

  // Aggregate all beams' footprint in spherical coordinates into one
  AxisAlignedPatch2d beamsSphericalFootprint;
  for (const auto & beam : this->dataPtr->beams)
  {
    beamsSphericalFootprint.Merge(beam.SphericalFootprint());
  }
  // Rendering sensors' FOV must be symmetric about its main axis
  beamsSphericalFootprint.Merge(beamsSphericalFootprint.Flip());

  // Configure depth sensor to cover the beams footprint
  // with configured resolution
  this->dataPtr->resolution  =
      arrangementElement->Get<double>("resolution", 0.01).first;
  ignmsg << "Setting beams' resolution to " << this->dataPtr->resolution
         << " m at a 1 m distance for [" << this->Name() << "] sensor."
         << std::endl;

  this->dataPtr->depthSensor->SetAngleMin(beamsSphericalFootprint.XMin());
  this->dataPtr->depthSensor->SetAngleMax(beamsSphericalFootprint.XMax());
  auto horizontalRayCount = static_cast<unsigned int>(
      std::ceil(beamsSphericalFootprint.XSize() /
                this->dataPtr->resolution));
  if (horizontalRayCount % 2 == 0) ++horizontalRayCount;  // ensure odd
  this->dataPtr->depthSensor->SetRayCount(horizontalRayCount);

  this->dataPtr->depthSensor->SetVerticalAngleMin(
      beamsSphericalFootprint.YMin());
  this->dataPtr->depthSensor->SetVerticalAngleMax(
      beamsSphericalFootprint.YMax());
  auto verticalRayCount = static_cast<unsigned int>(
      std::ceil(beamsSphericalFootprint.YSize() /
                this->dataPtr->resolution));
  if (verticalRayCount % 2 == 0) ++verticalRayCount;  // ensure odd
  this->dataPtr->depthSensor->SetVerticalRayCount(verticalRayCount);

  auto & intrinsics = this->dataPtr->depthSensorIntrinsics;
  intrinsics.offset.X(beamsSphericalFootprint.XMin());
  intrinsics.offset.Y(beamsSphericalFootprint.YMin());
  intrinsics.step.X(beamsSphericalFootprint.XSize() / (horizontalRayCount - 1));
  intrinsics.step.Y(beamsSphericalFootprint.YSize() / (verticalRayCount - 1));

  // Pre-compute scan indices covered by beam spherical
  // footprints for speed during scan iteration
  this->dataPtr->beamScanPatches.clear();
  for (const auto & beam : this->dataPtr->beams)
  {
    this->dataPtr->beamScanPatches.push_back(AxisAlignedPatch2i{
        (beam.SphericalFootprint() - intrinsics.offset) / intrinsics.step});
  }

  const auto minimumRange =
      this->dataPtr->sensorSdf->Get<double>("minimum_range", 0.1).first;
  ignmsg << "Setting minimum range to " << minimumRange
         << " m for [" << this->Name() << "] sensor." << std::endl;
  this->dataPtr->depthSensor->SetNearClipPlane(minimumRange);

  const auto maximumRange =
      this->dataPtr->sensorSdf->Get<double>("maximum_range", 100.).first;
  ignmsg << "Setting maximum range to " << maximumRange
         << " m for [" << this->Name() << "] sensor." << std::endl;
  this->dataPtr->depthSensor->SetFarClipPlane(maximumRange);

  this->dataPtr->depthSensor->SetVisibilityMask(~IGN_VISIBILITY_GUI);
  this->dataPtr->depthSensor->SetClamp(false);

  if (this->dataPtr->sensorSdf->HasElement("noise"))
  {
    ignmsg << "Setting noise model for "
           << "[" << this->Name() << "] sensor."
           << std::endl;
    auto noiseElement =
        this->dataPtr->sensorSdf->GetElement("noise");
    using NoiseFactory = gz::sensors::NoiseFactory;
    this->dataPtr->noiseModel =
        NoiseFactory::NewNoiseModel(noiseElement, "gpu_lidar");
    if (!this->dataPtr->noiseModel)
    {
      return false;
    }
  }

  this->AddSensor(this->dataPtr->depthSensor);

  this->dataPtr->imageSensor =
      this->Scene()->CreateCamera(this->Name() + "_image_sensor");
  if (!this->dataPtr->imageSensor)
  {
    ignerr << "Failed to create image sensor for "
           << "for [" << this->Name() << "] sensor."
           << std::endl;
    return false;
  }

  this->dataPtr->imageSensor->SetImageWidth(horizontalRayCount);
  this->dataPtr->imageSensor->SetImageHeight(verticalRayCount);

  this->dataPtr->imageSensor->SetNearClipPlane(minimumRange);
  this->dataPtr->imageSensor->SetFarClipPlane(maximumRange);
  this->dataPtr->imageSensor->SetAntiAliasing(2);

  this->dataPtr->imageSensor->SetAspectRatio(
      beamsSphericalFootprint.XSize() / beamsSphericalFootprint.YSize());
  this->dataPtr->imageSensor->SetHFOV(beamsSphericalFootprint.XSize());
  this->dataPtr->imageSensor->SetVisibilityMask(~IGN_VISIBILITY_GUI);

  this->AddSensor(this->dataPtr->imageSensor);

  this->dataPtr->referenceFrameTransform =
      this->dataPtr->sensorSdf->Get<gz::math::Pose3d>(
          "reference_frame", gz::math::Pose3d{}).first;

  this->dataPtr->depthConnection =
      this->dataPtr->depthSensor->ConnectNewGpuRaysFrame(
      std::bind(&DopplerVelocityLog::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  constexpr double epsilon = std::numeric_limits<double>::epsilon();
  const std::chrono::steady_clock::duration lifetime =
      std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::duration<double>(this->UpdateRate() > epsilon ?
                                        1. / this->UpdateRate() : 0.001));

  this->dataPtr->visualizeBeamLobes =
      arrangementElement->Get<bool>("visualize", false).first;
  if (this->dataPtr->visualizeBeamLobes)
  {
    ignmsg << "Enabling beam lobes' visual aids for "
           << "[" << this->Name() << "] sensor." << std::endl;

    for (const AcousticBeam & beam : this->dataPtr->beams)
    {
      const double angularResolution =
          this->dataPtr->resolution / beam.NormalizedRadius();
      const int lobeNumTriangles =
          static_cast<int>(std::ceil(2. * IGN_PI / angularResolution));

      auto * lobeConeMarkerMessage = this->dataPtr->beamLobesMessage.add_marker();
      lobeConeMarkerMessage->set_id(2 * beam.Id());
      lobeConeMarkerMessage->set_ns(this->Name() + "::lobes");
      lobeConeMarkerMessage->set_action(gz::msgs::Marker::ADD_MODIFY);
      lobeConeMarkerMessage->set_type(gz::msgs::Marker::TRIANGLE_FAN);
      lobeConeMarkerMessage->set_visibility(gz::msgs::Marker::GUI);
      *lobeConeMarkerMessage->mutable_lifetime() = gz::msgs::Convert(lifetime);
      auto * lobeConeMaterialMessage = lobeConeMarkerMessage->mutable_material();
      constexpr gz::math::Color lobeConeColor(1., 0., 0., 0.35);
      gz::msgs::Set(lobeConeMaterialMessage->mutable_ambient(), lobeConeColor);
      gz::msgs::Set(lobeConeMaterialMessage->mutable_diffuse(), lobeConeColor);
      gz::msgs::Set(lobeConeMaterialMessage->mutable_emissive(), lobeConeColor);
      gz::msgs::Set(lobeConeMarkerMessage->add_point(), gz::math::Vector3d::Zero);
      for (size_t i = 0; i < lobeNumTriangles; ++i)
      {
        gz::msgs::Set(
            lobeConeMarkerMessage->add_point(), gz::math::Vector3d{
              1.,
              -beam.NormalizedRadius() * std::cos(i * angularResolution),
              beam.NormalizedRadius() * std::sin(i * angularResolution)
            });
      }
      gz::msgs::Set(
          lobeConeMarkerMessage->add_point(),
          gz::math::Vector3d{1., -beam.NormalizedRadius(), 0.});

      auto * lobeCapMarkerMessage =
          this->dataPtr->beamLobesMessage.add_marker();
      lobeCapMarkerMessage->set_id(2 * beam.Id() + 1);
      lobeCapMarkerMessage->set_ns(this->Name() + "::lobes");
      lobeCapMarkerMessage->set_action(gz::msgs::Marker::ADD_MODIFY);
      lobeCapMarkerMessage->set_type(gz::msgs::Marker::TRIANGLE_FAN);
      lobeCapMarkerMessage->set_visibility(gz::msgs::Marker::GUI);
      *lobeCapMarkerMessage->mutable_lifetime() = gz::msgs::Convert(lifetime);
      auto * lobeCapMaterialMessage = lobeCapMarkerMessage->mutable_material();
      constexpr gz::math::Color lobeCapColor(1., 0., 0., 0.65);
      gz::msgs::Set(lobeCapMaterialMessage->mutable_ambient(), lobeCapColor);
      gz::msgs::Set(lobeCapMaterialMessage->mutable_diffuse(), lobeCapColor);
      gz::msgs::Set(lobeCapMaterialMessage->mutable_emissive(), lobeCapColor);
      gz::msgs::Set(
          lobeCapMarkerMessage->add_point(), gz::math::Vector3d{1., 0., 0.});
      for (size_t i = 0; i < lobeNumTriangles; ++i)
      {
        gz::msgs::Set(
            lobeCapMarkerMessage->add_point(), gz::math::Vector3d{
              1.,
              beam.NormalizedRadius() * std::cos(i * angularResolution),
              beam.NormalizedRadius() * std::sin(i * angularResolution)
            });
      }
      gz::msgs::Set(
          lobeCapMarkerMessage->add_point(),
          gz::math::Vector3d{1., beam.NormalizedRadius(), 0.});
    }
  }

  this->dataPtr->visualizeBeamReflections =
      this->dataPtr->sensorSdf->Get<bool>("visualize", false).first;
  if (this->dataPtr->visualizeBeamReflections)
  {
    ignmsg << "Enabling beam reflections' visual aids for "
           << "[" << this->Name() << "] sensor." << std::endl;

    for (const AcousticBeam & beam : this->dataPtr->beams)
    {
      auto * refConeMarkerMessage =
          this->dataPtr->beamReflectionsMessage.add_marker();
      refConeMarkerMessage->set_id(2 * beam.Id());
      refConeMarkerMessage->set_ns(this->Name() + "::reflections");
      refConeMarkerMessage->set_action(gz::msgs::Marker::ADD_MODIFY);
      refConeMarkerMessage->set_type(gz::msgs::Marker::TRIANGLE_FAN);
      refConeMarkerMessage->set_visibility(gz::msgs::Marker::GUI);
      *refConeMarkerMessage->mutable_lifetime() = gz::msgs::Convert(lifetime);
      auto * refConeMaterialMessage = refConeMarkerMessage->mutable_material();
      constexpr gz::math::Color refConeColor(0., 1., 0.);
      gz::msgs::Set(refConeMaterialMessage->mutable_ambient(), refConeColor);
      gz::msgs::Set(refConeMaterialMessage->mutable_diffuse(), refConeColor);
      gz::msgs::Set(refConeMaterialMessage->mutable_emissive(), refConeColor);

      gz::msgs::Set(
          refConeMarkerMessage->add_point(), gz::math::Vector3d::Zero);
      for (size_t i = 0; i < 4; ++i)
      {
        gz::msgs::Set(
            refConeMarkerMessage->add_point(), gz::math::Vector3d{
              1.,
              -this->dataPtr->resolution * std::cos(i * M_PI / 2.),
              this->dataPtr->resolution * std::sin(i * M_PI / 2.)
            });
      }
      gz::msgs::Set(
          refConeMarkerMessage->add_point(),
          gz::math::Vector3d{1., -this->dataPtr->resolution, 0.});

      auto * refCapMarkerMessage =
          this->dataPtr->beamReflectionsMessage.add_marker();
      refCapMarkerMessage->set_id(2 * beam.Id() + 1);
      refCapMarkerMessage->set_ns(this->Name() + "::reflections");
      refCapMarkerMessage->set_action(gz::msgs::Marker::ADD_MODIFY);
      refCapMarkerMessage->set_type(gz::msgs::Marker::TRIANGLE_FAN);
      refCapMarkerMessage->set_visibility(gz::msgs::Marker::GUI);
      *refCapMarkerMessage->mutable_lifetime() = gz::msgs::Convert(lifetime);
      auto * refCapMaterialMessage = refCapMarkerMessage->mutable_material();
      constexpr gz::math::Color refCapColor(0., 1., 0.);
      gz::msgs::Set(refCapMaterialMessage->mutable_ambient(), refCapColor);
      gz::msgs::Set(refCapMaterialMessage->mutable_diffuse(), refCapColor);
      gz::msgs::Set(refCapMaterialMessage->mutable_emissive(), refCapColor);

      gz::msgs::Set(
          refCapMarkerMessage->add_point(), gz::math::Vector3d{1., 0., 0.});
      for (size_t i = 0; i < 4; ++i)
      {
        gz::msgs::Set(
            refCapMarkerMessage->add_point(), gz::math::Vector3d{
              1.,
              this->dataPtr->resolution * std::cos(i * M_PI / 2.),
              this->dataPtr->resolution * std::sin(i * M_PI / 2.)
            });
      }
      gz::msgs::Set(
          refCapMarkerMessage->add_point(),
          gz::math::Vector3d{1., this->dataPtr->resolution, 0.});
    }
  }

  ignmsg << "Initialized [" << this->Name() << "] sensor." << std::endl;

  return true;
}

//////////////////////////////////////////////////
std::vector<gz::rendering::SensorPtr>
DopplerVelocityLog::RenderingSensors() const
{
  return {this->dataPtr->depthSensor, this->dataPtr->imageSensor};
}

//////////////////////////////////////////////////
void DopplerVelocityLog::OnNewFrame(
    const float *_scan,
    unsigned int _width,
    unsigned int _height,
    unsigned int _channels,
    const std::string & /*_format*/)
{
  const auto & intrinsics = this->dataPtr->depthSensorIntrinsics;

  for (size_t i = 0; i < this->dataPtr->beams.size(); ++i)
  {
    const AxisAlignedPatch2i & beamScanPatch =
        this->dataPtr->beamScanPatches[i];
    const AcousticBeam & beam = this->dataPtr->beams[i];

    // Clear existing target, if any
    std::optional<Target> & beamTarget =
        this->dataPtr->beamTargets[i];
    beamTarget.reset();

    // Iterate over the beam solid angle in camera coordinates
    for (auto v = beamScanPatch.YMin(); v < beamScanPatch.YMax(); ++v)
    {
      assert(v >= 0 && v < _height);
      const gz::math::Angle inclination =
          v * intrinsics.step.Y() + intrinsics.offset.Y();

      for (auto u = beamScanPatch.XMin(); u < beamScanPatch.XMax(); ++u)
      {
        assert(u >= 0 && u < _width);

        const float range =
            this->dataPtr->noiseModel->Apply(
                _scan[(u + v * _width) * _channels]);
        if (!std::isfinite(range)) continue;

        const gz::math::Angle azimuth =
            u * intrinsics.step.X() + intrinsics.offset.X();

        // Convert to cartesian coordinates in the acoustic beams' frame
        const auto point = range * gz::math::Vector3d{
          std::cos(inclination.Radian()) * std::cos(azimuth.Radian()),
          std::cos(inclination.Radian()) * std::sin(azimuth.Radian()),
          std::sin(inclination.Radian())
        };

        // Track point if (a) it effectively lies within the
        // beam's aperture and (b) it is the closest seen so far
        const gz::math::Angle angle = std::acos(
            point.Normalized().Dot(beam.Axis()));
        if (angle < beam.ApertureAngle() / 2.)
        {
          if (beamTarget)
          {
            if (beamTarget->pose.Pos().Length() > point.Length())
            {
              beamTarget->pose.Pos() = point;
            }
          }
          else
          {
            beamTarget = {
              gz::math::Pose3d{
                point,
                gz::math::Quaterniond::Identity},
              gz::sim::kNullEntity
            };
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void DopplerVelocityLog::SetScene(gz::rendering::ScenePtr _scene)
{
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    // TODO(anyone) Remove camera from scene
    this->dataPtr->depthSensor = nullptr;
    this->dataPtr->imageSensor = nullptr;
    RenderingSensor::SetScene(_scene);
    if (this->dataPtr->initialized)
    {
      if (!this->CreateRenderingSensors())
      {
        ignerr << "Failed to recreate rendering sensors for "
               << "[" << this->Name() << "]. Aborting load."
               << std::endl;
      }
    }
  }
}

//////////////////////////////////////////////////
void DopplerVelocityLog::SetWorldState(const WorldKinematicState &_state)
{
  this->dataPtr->worldState = _state;
}

//////////////////////////////////////////////////
void DopplerVelocityLog::SetEntity(gz::sim::Entity _entityId)
{
  this->dataPtr->entityId = _entityId;
}

//////////////////////////////////////////////////
bool DopplerVelocityLog::Update(const std::chrono::steady_clock::duration &)
{
  IGN_PROFILE("DopplerVelocityLog::Update");
  if (!this->dataPtr->initialized ||
      this->dataPtr->entityId == gz::sim::kNullEntity)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->pub.HasConnections())
  {
    if (this->dataPtr->generatingData)
    {
      igndbg << "Disabling data generation for sensor "
             << "[" << this->Name() << "] as no subscribers"
             << " were found." << std::endl;
      this->dataPtr->generatingData = false;
    }

    return false;
  }
  else
  {
    if (!this->dataPtr->generatingData)
    {
      igndbg << "Enabling data generation for sensor "
             << "[" << this->Name() << "] as some subscribers "
             << "were found." << std::endl;
      this->dataPtr->generatingData = true;
    }
  }

  const gz::math::Pose3d beamsFramePose =
      this->dataPtr->beamsFrameTransform * this->Pose();
  this->dataPtr->depthSensor->SetLocalPose(beamsFramePose);
  this->dataPtr->imageSensor->SetLocalPose(beamsFramePose);

  // Generate sensor data
  this->Render();

  return true;
}

//////////////////////////////////////////////////
void DopplerVelocityLog::PostUpdate(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("DopplerVelocityLog::PostUpdate");

  if (!this->dataPtr->generatingData)
  {
    // Nothing to publish
    return;
  }

  // Used to populate visual aids
  const std::string parentName =
      this->dataPtr->depthSensor->Parent()->Name();
  const gz::math::Pose3d beamsFramePose =
      this->dataPtr->beamsFrameTransform * this->Pose();

  // Populate and publish velocity tracking estimates
  DVLVelocityTracking trackingMessage;
  auto * headerMessage = trackingMessage.mutable_header();
  *headerMessage->mutable_stamp() = gz::msgs::Convert(_now);
  this->AddSequence(headerMessage, "doppler_velocity_log");
  trackingMessage.set_type(this->dataPtr->dvlType);

  size_t numBeamsLocked = 0;
  double targetRange = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd beamBasis(this->dataPtr->beams.size(), 3);
  Eigen::VectorXd beamSpeeds(this->dataPtr->beams.size());
  const EntityKinematicState & sensorStateInWorldFrame =
      this->dataPtr->worldState.at(this->dataPtr->entityId);
  for (size_t i = 0; i < this->dataPtr->beams.size(); ++i)
  {
    const AcousticBeam & beam = this->dataPtr->beams[i];
    auto * beamMessage = trackingMessage.add_beams();
    beamMessage->set_id(beam.Id());

    auto & beamTarget = this->dataPtr->beamTargets[i];
    if (beamTarget)
    {
      // TODO(hidmic): use shader to fetch target entity id
      const gz::math::Vector2i pixel =
          this->dataPtr->imageSensor->Project(beamTarget->pose.Pos());
      auto visual = this->dataPtr->imageSensor->VisualAt(pixel);
      if (visual)
      {
        if (visual->HasUserData("gazebo-entity"))
        {
          auto user_data = visual->UserData("gazebo-entity");
          beamTarget->entity = std::get<uint64_t>(user_data);
        }
        else
        {
          igndbg << "No entity associated to [" << visual->Name() << "] visual."
                 << " Assuming it is static w.r.t. the world." << std::endl;
        }
      }

      const double beamRange = beamTarget->pose.Pos().Length();
      auto * beamRangeMessage = beamMessage->mutable_range();
      beamRangeMessage->set_mean(beamRange);

      if (this->dataPtr->visualizeBeamLobes)
      {
        const double length =
            beamTarget->pose.Pos().Dot(beam.Axis());
        const auto scale = length * gz::math::Vector3d::One;
        auto * lobeConeMarkerMessage =
            this->dataPtr->beamLobesMessage.mutable_marker(2 * i);
        lobeConeMarkerMessage->set_parent(parentName);
        gz::msgs::Set(lobeConeMarkerMessage->mutable_scale(), scale);
        gz::msgs::Set(lobeConeMarkerMessage->mutable_pose(),
                      beamsFramePose * beam.Transform());
        auto * lobeCapMarkerMessage =
            this->dataPtr->beamLobesMessage.mutable_marker(2 * i + 1);
        lobeCapMarkerMessage->set_parent(parentName);
        gz::msgs::Set(lobeCapMarkerMessage->mutable_scale(), scale);
        gz::msgs::Set(lobeCapMarkerMessage->mutable_pose(),
                      beamsFramePose * beam.Transform());
      }

      if (this->dataPtr->visualizeBeamReflections)
      {
        gz::math::Pose3d transform;
        transform.Rot().SetFrom2Axes(
            beam.Axis(), beamTarget->pose.Pos());
        const auto scale = beamRange * gz::math::Vector3d::One;
        auto * refConeMarkerMessage =
            this->dataPtr->beamReflectionsMessage.mutable_marker(2 * i);
        refConeMarkerMessage->set_parent(parentName);
        gz::msgs::Set(refConeMarkerMessage->mutable_scale(), scale);
        gz::msgs::Set(refConeMarkerMessage->mutable_pose(),
                      beamsFramePose * transform * beam.Transform());
        auto * refCapMarkerMessage =
            this->dataPtr->beamReflectionsMessage.mutable_marker(2 * i + 1);
        refCapMarkerMessage->set_parent(parentName);
        gz::msgs::Set(refCapMarkerMessage->mutable_scale(), scale);
        gz::msgs::Set(refCapMarkerMessage->mutable_pose(),
                      beamsFramePose * transform * beam.Transform());
      }

      // Use shortest beam range as target range
      targetRange = std::min(targetRange, beamRange);

      // Compute target speed as it would have been measured by the beam
      const EntityKinematicState & targetEntityStateInWorldFrame =
          this->dataPtr->worldState[beamTarget->entity];

      const gz::math::Pose3d targetPoseInWorldFrame =
          sensorStateInWorldFrame.pose *
          this->dataPtr->beamsFrameTransform *
          beamTarget->pose;

      const gz::math::Vector3d targetVelocityInWorldFrame =
          targetEntityStateInWorldFrame.linearVelocity +
          targetEntityStateInWorldFrame.angularVelocity.Cross(
              targetPoseInWorldFrame.Pos() -
              targetEntityStateInWorldFrame.pose.Pos());

      const gz::math::Vector3d relativeSensorVelocityInSensorFrame =
          sensorStateInWorldFrame.pose.Rot().RotateVectorReverse(
              sensorStateInWorldFrame.linearVelocity -
              targetVelocityInWorldFrame);

      const gz::math::Vector3d beamAxisInSensorFrame =
          this->dataPtr->beamsFrameTransform.Rot() * beam.Axis();
      const double beamSpeed =
          relativeSensorVelocityInSensorFrame.Dot(beamAxisInSensorFrame);

      auto * beamVelocityMessage = beamMessage->mutable_velocity();
      beamVelocityMessage->set_reference(
          DVLKinematicEstimate::DVL_REFERENCE_SHIP);
      *beamVelocityMessage->mutable_mean() = gz::msgs::Convert(
          this->dataPtr->referenceFrameTransform.Rot().Inverse() *
          beamAxisInSensorFrame * beamSpeed);

      // Track sensor speed w.r.t target as measured by the beam
      // for least squares estimation of sensor velocity
      beamBasis(numBeamsLocked, 0) = beamAxisInSensorFrame.X();
      beamBasis(numBeamsLocked, 1) = beamAxisInSensorFrame.Y();
      beamBasis(numBeamsLocked, 2) = beamAxisInSensorFrame.Z();
      beamSpeeds(numBeamsLocked) = beamSpeed;
      ++numBeamsLocked;
    }
    beamMessage->set_locked(beamTarget.has_value());
    if (this->dataPtr->visualizeBeamLobes)
    {
      const auto action =
          beamTarget.has_value() ?
          gz::msgs::Marker::ADD_MODIFY :
          gz::msgs::Marker::DELETE_MARKER;
      auto * lobeConeMarkerMessage =
          this->dataPtr->beamLobesMessage
          .mutable_marker(2 * i);
      lobeConeMarkerMessage->set_action(action);
      auto * lobeCapMarkerMessage =
          this->dataPtr->beamLobesMessage
          .mutable_marker(2 * i + 1);
      lobeCapMarkerMessage->set_action(action);
    }
    if (this->dataPtr->visualizeBeamReflections)
    {
      const auto action =
          beamTarget.has_value() ?
          gz::msgs::Marker::ADD_MODIFY :
          gz::msgs::Marker::DELETE_MARKER;
      auto * reflectionConeMarkerMessage =
          this->dataPtr->beamReflectionsMessage
          .mutable_marker(2 * i);
      reflectionConeMarkerMessage->set_action(action);
      auto * reflectionCapMarkerMessage =
          this->dataPtr->beamReflectionsMessage
          .mutable_marker(2 * i + 1);
      reflectionCapMarkerMessage->set_action(action);
    }
  }
  if (numBeamsLocked >= 3)
  {
    constexpr auto svdFlags =
        Eigen::ComputeThinU | Eigen::ComputeThinV;
    const Eigen::Vector3d estimate =
        beamBasis.topRows(numBeamsLocked).jacobiSvd(svdFlags)
        .solve(beamSpeeds.head(numBeamsLocked));
    const gz::math::Vector3d velocityInSensorFrame{
      estimate.x(), estimate.y(), estimate.z()};
    auto * velocityMessage = trackingMessage.mutable_velocity();
    velocityMessage->set_reference(
        DVLKinematicEstimate::DVL_REFERENCE_SHIP);
    *velocityMessage->mutable_mean() = gz::msgs::Convert(
        this->dataPtr->referenceFrameTransform.Rot().Inverse() *
        velocityInSensorFrame);
  }
  auto * targetMessage = trackingMessage.mutable_target();
  targetMessage->set_type(DVLTrackingTarget::DVL_TARGET_BOTTOM);
  if (numBeamsLocked > 0)
  {
    auto * rangeMessage = targetMessage->mutable_range();
    rangeMessage->set_mean(targetRange);
  }
  trackingMessage.set_status(0);

  this->dataPtr->pub.Publish(trackingMessage);

  if (this->dataPtr->visualizeBeamLobes)
  {
    headerMessage = this->dataPtr->beamLobesMessage.mutable_header();
    *headerMessage->mutable_stamp() = gz::msgs::Convert(_now);
    this->AddSequence(headerMessage, "doppler_velocity_log_beam_lobes");

    bool result;
    gz::msgs::Boolean reply;
    constexpr unsigned int timeout_ms = 1000u;
    bool outcome = this->dataPtr->node.Request(
        "/marker_array",
        this->dataPtr->beamLobesMessage,
        timeout_ms, reply, result);
    if (!outcome || !result || !reply.data())
    {
      ignwarn << "Failed to render beam lobes' visual "
              << "aids for [" << this->Name() << "] sensor."
              << std::endl;
    }
  }

  if (this->dataPtr->visualizeBeamReflections)
  {
    headerMessage = this->dataPtr->beamReflectionsMessage.mutable_header();
    *headerMessage->mutable_stamp() = gz::msgs::Convert(_now);
    this->AddSequence(headerMessage, "doppler_velocity_log_beam_reflections");

    bool result;
    gz::msgs::Boolean reply;
    constexpr unsigned int timeout_ms = 1000u;
    bool outcome = this->dataPtr->node.Request(
        "/marker_array",
        this->dataPtr->beamReflectionsMessage,
        timeout_ms, reply, result);
    if (!outcome || !result || !reply.data())
    {
      ignwarn << "Failed to render beam reflections' visual "
              << "aids for [" << this->Name() << "] sensor."
              << std::endl;
    }
  }
}

}
