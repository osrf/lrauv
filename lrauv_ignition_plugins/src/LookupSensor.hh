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

#ifndef TETHYS_LOOKUPSENSOR_
#define TETHYS_LOOKUPSENSOR_

#include <ignition/common/Console.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/SensorTypes.hh>
#include <ignition/transport/Node.hh>
#include <ignition/sensors/Util.hh>

namespace tethys
{

// Macro to help creating new sensor types. This is needed to pass typeStr at
// compile time.
//
// Example usage:
//
// LOOKUP_SENSOR(SalinitySensor, double, salinity);
//
// className: Name of resulting class
// dataType: Type of data that the sensor will hold
// typeStr: String that uniquely identifies the sensor
#define LOOKUP_SENSOR(className, dataType, typeStr)\
  const char LS_##typeStr[]{#typeStr};\
  using className = LookupSensor<dataType, LS_##typeStr>;

/// \brief Generic sensor type for sensors that look up a value in a table.
/// \tparam DataType Type of data that the sensor looks up, such as double.
/// \tparam _typeStr The string representation of the sensor, such as "salinity"
template <typename DataType, const char *_typeStr>
class LookupSensor : public ignition::sensors::Sensor
{
  /// \brief Load the sensor with SDF parameters.
  /// \param[in] _sdf SDF Sensor parameters.
  /// \return True if loading was successful
  public: virtual bool Load(const sdf::Sensor &_sdf) override;

  /// \brief Update the sensor and generate data
  /// \param[in] _now Simulation time
  /// \return True if the update was successful
  public: virtual bool Update(
    const std::chrono::steady_clock::duration &_now) override;

  /// \brief Set the latest data for the sensor.
  /// \param[in] _data Latest data.
  public: void SetData(DataType _data);

  /// \brief String that uniquely identifies the sensor.
  public: static constexpr char const *kTypeStr{_typeStr};

  /// \brief Noise that will be applied to the sensor data
  protected: ignition::sensors::NoisePtr noise{nullptr};

  /// \brief Node for communication
  protected: ignition::transport::Node node;

  /// \brief Publishes sensor data
  protected: ignition::transport::Node::Publisher pub;

  /// \brief Latest data
  protected: DataType data;
};

//////////////////////////////////////////////////
template <typename DataType, const char *typeStr>
bool LookupSensor<DataType, typeStr>::Load(const sdf::Sensor &_sdf)
{
  auto type = ignition::sensors::customType(_sdf);
  if (kTypeStr != type)
  {
    ignerr << "Trying to load [" << kTypeStr << "] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  if constexpr (std::is_same<DataType, ignition::math::Vector3d>::value)
  {
    this->pub = this->node.Advertise<ignition::msgs::Vector3d>(this->Topic());
  }
  else if constexpr (std::is_same<DataType, float>::value)
  {
    this->pub = this->node.Advertise<ignition::msgs::Float>(this->Topic());
  }
  else
  {
    this->pub = this->node.Advertise<ignition::msgs::Double>(this->Topic());
  }

  std::string elementStr("ignition:" + std::string(kTypeStr));
  if (!_sdf.Element()->HasElement(elementStr))
  {
    igndbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement(elementStr);

  if (!customElem->HasElement("noise"))
  {
    igndbg << "No noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise noiseSdf;
  noiseSdf.Load(customElem->GetElement("noise"));
  this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    ignerr << "Failed to load noise." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
template <typename DataType, const char *typeStr>
bool LookupSensor<DataType, typeStr>::Update(
    const std::chrono::steady_clock::duration &_now)
{
  // Using constexpr because we can't partially specialize the template
  // (i.e. specialize DataType but not typeStr)
  if constexpr (std::is_same<DataType, ignition::math::Vector3d>::value)
  {
    ignition::msgs::Vector3d msg;

    msg.set_x(this->noise->Apply(this->data.X()));
    msg.set_y(this->noise->Apply(this->data.Y()));
    msg.set_z(this->noise->Apply(this->data.Z()));

    // Set header
    *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    this->AddSequence(msg.mutable_header());

    // Publish
    this->pub.Publish(msg);
  }
  else if constexpr (std::is_same<DataType, float>::value)
  {
    ignition::msgs::Float msg;

    msg.set_data(this->noise->Apply(this->data));

    // Set header
    *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    this->AddSequence(msg.mutable_header());

    // Publish
    this->pub.Publish(msg);
  }
  else
  {
    ignition::msgs::Double msg;

    if constexpr (std::is_same<DataType, ignition::math::Temperature>::value)
    {
      msg.set_data(this->noise->Apply(this->data.Celsius()));
    }
    else
    {
      msg.set_data(this->noise->Apply(this->data));
    }

    // Set header
    *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    this->AddSequence(msg.mutable_header());

    // Publish
    this->pub.Publish(msg);
  }

  return true;
}

//////////////////////////////////////////////////
template <typename DataType, const char *typeStr>
void LookupSensor<DataType, typeStr>::SetData(DataType _data)
{
  this->data = _data;
}
}
#endif
