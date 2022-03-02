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

using namespace tethys;

DVLSensor::Load(const sdf::Sensor &_sdf)
{

}

bool DVLSensor::Update(const std::chrono::steady_clock::duration &_now)
{
}

void DVLSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
}