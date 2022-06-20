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

#include <chrono>

#include <gz/sim/World.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include "TimeAnalysisPlugin.hh"

using namespace tethys;

void TimeAnalysisPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  ignmsg << "TimeAnalysisPlugin::Configure" << std::endl;

  // Subscribe to world stats for actual real time factor
  if (!this->node.Subscribe("/stats",
      &TimeAnalysisPlugin::RTFCallback, this))
  {
    ignerr << "Error subscribing to topic " << "[" << "/stats" << "]. "
      << std::endl;
    return;
  }

  // Get world name
  gz::sim::World world(_entity);
  if (!world.Valid(_ecm))
  {
    ignerr << "Time analysis plugin must be attached to the world."
           << std::endl;
    return;
  }

  // Service for setting max step size and RTF dynamically
  this->physicsCmdService = "/world/" + world.Name(_ecm).value() +
      "/set_physics";
  this->physicsCmdService = gz::transport::TopicUtils::AsValidTopic(
    this->physicsCmdService);
  if (this->physicsCmdService.empty())
  {
    ignerr << "Invalid physics command service topic provided" << std::endl;
    return;
  }

  std::cerr << "dt(ms),RTF" << std::endl;
}

void TimeAnalysisPlugin::RTFCallback(
  const gz::msgs::WorldStatistics &_msg)
{
  // Store RTF
  this->rtf = _msg.real_time_factor();
}

void TimeAnalysisPlugin::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &)
{
  // If paused or finished testing, do nothing
  // Use >, not >=, because if nextStepSizeIdx == array size, still need to run
  // last value in the array.
  if (_info.paused || nextStepSizeIdx > std::size(this->stepSizes))
    return;

  // Set a new max_step_size periodically
  // Do this at start of function, or else do it in Configure(), to overwrite
  // the max_step_size and real_time_factor in the SDF file.
  if (this->intervalStart.count() == 0 ||
     _info.realTime - this->intervalStart > this->interval)
  {
    // Set RTF to 0 for as fast as possible
    double nextRealTimeFactor = 0.0;

    // Go to next step size. Use == to only print once.
    if (nextStepSizeIdx == std::size(this->stepSizes))
    {
      ignmsg << "Data collection complete." << std::endl;
      // Increment again so it would not come into function to keep printing
      nextStepSizeIdx++;
      return;
    }
    double nextStepSize = this->stepSizes[nextStepSizeIdx];
    nextStepSizeIdx++;
    ignmsg << "Setting new max_step_size (dt) to " << nextStepSize << std::endl;

    std::function<void(const gz::msgs::Boolean &, const bool)> physCb =
        [](const gz::msgs::Boolean &/*_rep*/, const bool _result)
    {
      if (!_result)
        ignerr << "Error setting physics parameters" << std::endl;
    };

    // Set physics parameters dynamically
    gz::msgs::Physics req;
    req.set_max_step_size(nextStepSize);
    req.set_real_time_factor(nextRealTimeFactor);
    this->node.Request(this->physicsCmdService, req, physCb);

    this->intervalStart = _info.realTime;
  }

  // dt
  int dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    _info.dt).count();

  // At regular intervals, collect a data point.
  // Print raw text for quick dump to .csv file.
  // x,y
  if (_info.realTime - this->prevCollectTime > this->collectInterval)
  {
    std::cerr << dt_ms << "," << this->rtf << std::endl;
    this->prevCollectTime = _info.realTime;
  }
}

GZ_ADD_PLUGIN(
  tethys::TimeAnalysisPlugin,
  gz::sim::System,
  tethys::TimeAnalysisPlugin::ISystemConfigure,
  tethys::TimeAnalysisPlugin::ISystemPostUpdate)
