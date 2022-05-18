#ifndef TETHYS_DOPPLERVELOCITYLOG_HH_
#define TETHYS_DOPPLERVELOCITYLOG_HH_

#include <chrono>
#include <memory>
#include <unordered_map>

#include <gz/sim/Entity.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/RenderingSensor.hh>

namespace tethys
{

/// \brief Kinematic state for an entity in the world.
///
/// All quantities are defined w.r.t. the world frame.
struct EntityKinematicState
{
  gz::math::Pose3d pose;
  gz::math::Vector3d linearVelocity;
  gz::math::Vector3d angularVelocity;
};

/// \brief Kinematic state for all entities in the world.
using WorldKinematicState = std::unordered_map<
  gz::sim::Entity, EntityKinematicState>;

class DopplerVelocityLogPrivate;

/// \brief Doppler velocity log (DVL) sensor, built as a custom
/// rendering sensor to leverage GPU shaders for speed.
///
/// By default, this sensor uses an x-forward, y-left, z-up
/// coordinate frame to report all measurements. All acoustic
/// beams' lobes are expected to face downwards.
///
/// \verbatim
/// <sensor type="custom" ignition:type="dvl">
///   <ignition:dvl>
///     <arrangement degrees="true">
///       <beam id="1">
///         <aperture></aperture>
///         <rotation></rotation>
///         <tilt></tilt>
///       </beam>
///       <resolution></resolution>
///     </arrangement>
///     <noise type="gaussian">
///       <stddev></stddev>
///     </noise>
///     <minimum_range></minimum_range>
///     <maximum_range></maximum_range>
///     <reference_frame></reference_frame>
///   </ignition:dvl>
/// </sensor>
/// \endverbatim
///
/// - `<arrangement>` describes the arrangement of acoustic beams
/// in the DVL sensor frame. It may include a `degrees` attribute
/// to signal use of degrees instead of radians for all angles
/// within, defaulting to radians if left unspecified.
/// - `<arrangement><beam>` describes one acoustic beam in the
/// arrangement. May include an `id` attribute, defaulting to the
/// last specified id plus 1 if left unspecified (or 0 if it's the
/// first).
/// - `<arrangement><beam><aperture>` sets the aperture angle for
/// the acoustic beam's main lobe (modelled as a cone). Defaults
/// to 90 degrees if left unspecified.
/// - `<arrangement><beam><rotation>` sets the rotation angle of
/// the acoustic beam's symmetry axis about the sensor frame z
/// axis. Defaults to 0 degrees if left unspecified.
/// - `<arrangement><beam><tilt>` sets the inclination angle of
/// the acoustic beam's symmetry axis w.r.t. the sensor frame -z
/// axis (ie. rotation about the -y axis). Defaults to 0 degrees
/// if left unspecified.
/// - `<arrangement><resolution>` sets the resolution of the beam
/// arrangement at a 1 m distance. Defaults to 1 cm if left unspecified.
/// - `<noise>` sets the noise model for range measurements.
/// Defaults to none if left unspecified
/// - `<minimum_range>` sets a lower bound for range measurements.
/// Defaults to 1 cm if left unspecified.
/// - `<maximum_range>` sets an upper bound for range measurements.
/// Defaults to 100 m if left unspecified.
/// - `<reference_frame>` sets a transform from the sensor frame to the
/// reference frame in which all measurements are reported. Defaults to
/// the identity transform.
///
/// Note the tethys::DopplerVelocityLogSystem plugin must be
/// loaded for these custom sensors to be picked up and setup.
class DopplerVelocityLog : public gz::sensors::RenderingSensor
{
  public: DopplerVelocityLog();

  public: ~DopplerVelocityLog();

  /// Inherits documentation from parent class
  public: virtual bool Load(const sdf::Sensor &_sdf) override;

  /// Inherits documentation from parent class
  public: virtual bool Update(
    const std::chrono::steady_clock::duration &_now) override;

  /// Perform any sensor updates after the rendering pass
  public: virtual void PostUpdate(
    const std::chrono::steady_clock::duration &_now);

  /// Inherits documentation from parent class
  public: void SetScene(gz::rendering::ScenePtr _scene) override;

  /// \brief Set this sensor's entity ID (for world state lookup).
  public: void SetEntity(gz::sim::Entity entity);

  /// \brief Provide world `_state` to support DVL velocity estimates.
  public: void SetWorldState(const WorldKinematicState &_state);

  /// \brief Yield rendering sensors that underpin the implementation.
  ///
  /// \internal
  public: std::vector<gz::rendering::SensorPtr> RenderingSensors() const;

  /// \brief Create rendering sensors that underpin the implementation.
  ///
  /// \internal
  private: bool CreateRenderingSensors();

  /// \brief Callback for rendering sensor frames
  private: void OnNewFrame(
      const float *_scan, unsigned int _width,
      unsigned int _height, unsigned int _channels,
      const std::string & /*_format*/);

  private: std::unique_ptr<DopplerVelocityLogPrivate> dataPtr;
};

}  // namespace tethys

#endif //TETHYS_DOPPLERVELOCITYLOG_HH_
