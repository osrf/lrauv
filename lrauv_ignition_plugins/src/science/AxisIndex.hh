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

#ifndef __LRAUV_IGNITION_PLUGINS_SRC_SCIENCE_AXIS_INDEX_HH_
#define __LRAUV_IGNITION_PLUGINS_SRC_SCIENCE_AXIS_INDEX_HH_

#include <algorithm>
#include <map>
#include <optional>
namespace tethys {
  class AxisIndex
  {
    private: std::map<double, std::size_t> axisIndex;

    private: int numIndices{0};

    public: void AddIndexIfNotFound(double _value)
    {
      if (axisIndex.find(_value) == axisIndex.end())
      {
        axisIndex[_value] = numIndices;
        numIndices++;
      }
    }

    public: std::size_t GetNumUniqueIndices()
    {
      return axisIndex.size();
    }

    public: std::optional<std::size_t> GetIndex(double _value)
    {
      if (axisIndex.find(_value) == axisIndex.end())
      {
        return std::nullopt;
      }
      else
      {
        return axisIndex[_value];
      }
    }

    public: std::vector<std::size_t> GetInterpolators(double _value,
      double _tol=1e-6)
    {
      // Performs a BST to find the first element that is greater than or equal
      // to the value.
      auto it = axisIndex.lower_bound(_value);
      if (it == axisIndex.end())
      {
        // Out of range
        return {};
      }
      else if (abs(it->first - _value) < _tol)
      {
        // Exact match
        return {it->second};
      }
      else if (it == axisIndex.begin())
      {
        // Below range
        return {};
      }
      else
      {
        // Interpolate
        auto itPrev = it;
        itPrev--;
        return {itPrev->second, it->second};
      }
    }
  };
}

#endif