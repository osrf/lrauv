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
#ifndef TIME_ANALYSIS_PLUGIN_H_
#define TIME_ANALYSIS_PLUGIN_H_

#include <chrono>

#include <ignition/gazebo/System.hh>
#include <ignition/msgs/world_stats.pb.h>
#include <ignition/transport/Node.hh>

namespace tethys
{
  class TimeAnalysisPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
    // Documentation inherited
    public: void Configure(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// Callback function for world stats containing real time factor
    /// \param[in] _msg world stats message
    public: void RTFCallback(const ignition::msgs::WorldStatistics &_msg);

    /// Transport node for message passing
    private: ignition::transport::Node node;

    /// Service name for setting physics time step and RTF
    private: std::string physicsCmdService{""};

    /// Interval for each step size
    private: std::chrono::steady_clock::duration interval =
      std::chrono::seconds(5);

    /// Starting timestamp of current interval
    private: std::chrono::steady_clock::duration intervalStart =
      std::chrono::steady_clock::duration::zero();

    /// Interval for each data point collection
    private: std::chrono::steady_clock::duration collectInterval =
      std::chrono::milliseconds(100);

    /// Previous collection time
    private: std::chrono::steady_clock::duration prevCollectTime =
      std::chrono::steady_clock::duration::zero();

    /// Values of max step sizes to test
    private: double stepSizes[30] = {
      // Too slow
      0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009,
      // Desired
      0.010, 0.011, 0.012, 0.013, 0.014, 0.015, 0.016, 0.017, 0.018, 0.019, 0.020,
      // Too fast, will crash if keep incrementing
      0.021, 0.022, 0.023, 0.024, 0.025, 0.026, 0.027, 0.028, 0.029, 0.030
      //0.025, 0.03,
    };

    /// Next index for stepSizes array
    private: int nextStepSizeIdx = 0;

    /// Latest real time factor
    private: double rtf = 0.0;
  };
}

#endif /*TIME_ANALYSIS_PLUGIN_H_*/
