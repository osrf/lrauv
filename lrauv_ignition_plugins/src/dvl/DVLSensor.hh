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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#ifndef LRAUV_IGNITION_PLUGINS_DVL_DVLSENSOR_HH
#define LRAUV_IGNITION_PLUGINS_DVL_DVLSENSOR_HH

#include <ignition/sensors/RenderingSensor.hh>

#include <ignition/transport/Node.hh>

namespace tethys {
  /// \brief Doppler velocity log sensor
  class DVLSensor : public ignition::sensors::RenderingSensor
  {
    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successfull
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Makes possible to change sensor scene
    /// \param[in] _scene used with the sensor
    public: virtual void SetScene(ignition::rendering::ScenePtr _scene) override;

    /// \brief Node for communication
    private: ignition::transport::Node node;

    /// \brief Publishes sensor data
    private: ignition::transport::Node::Publisher pub;
};
}

#endif