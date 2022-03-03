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

#include "DVLSensor.hh"

#include "lrauv_dvl_message.pb.h"

#include <ignition/common.hh>
#include <ignition/sensors.hh>

using namespace tethys;

bool DVLSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = ignition::sensors::customType(_sdf);
  if ("dvl" != type)
  {
    ignerr << "Trying to load [dvl] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::DVL>(this->Topic());

  if (!_sdf.Element()->HasElement("ignition:dvl"))
  {
    igndbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  /*
  auto customElem = _sdf.Element()->GetElement("ignition:dvl");

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
  */

  return true;
}

bool DVLSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  ignition::msgs::DVL dvlMsg;

  this->pub.Publish(dvlMsg);
}

void DVLSensor::SetScene(ignition::rendering::ScenePtr _scene)
{

}
