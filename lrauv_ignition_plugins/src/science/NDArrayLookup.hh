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

#ifndef __LRAUV_IGNITION_PLUGINS_SRC_SCIENCE_NDARRAYLOOKUP_HH_
#define __LRAUV_IGNITION_PLUGINS_SRC_SCIENCE_NDARRAYLOOKUP_HH_

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include <ignition/math/Vector3.hh>
#include <pcl/common/common.h>

#include "AxisIndex.hh"
namespace tethys {

  class VolumetricScalarField
  {
    /// Get the index along the given axis
    public: AxisIndex z_indices_by_depth;

    public: AxisIndex x_indices_by_lat;

    public: AxisIndex y_indices_by_lon;

    private: std::vector<std::vector<std::vector<std::optional<std::size_t>>>> index_table;

    /// \brief Constructor
    public: VolumetricScalarField(
      const pcl::PointCloud<pcl::PointXYZ> &_cloud)
    {
      pcl::getMinMax3D (_cloud, minPt, maxPt);
      // NOTE: This part of the code assumes an exact grid of points.
      // The grid may be distorted or the stride between different points may
      // not be the same, but fundamentally the data is structured in a grid.
      // It keeps track of the axis indices for each point in the grid.
      for(auto pt: _cloud)
      {
        x_indices_by_lat.AddIndexIfNotFound(pt.x);
        y_indices_by_lon.AddIndexIfNotFound(pt.y);
        z_indices_by_depth.AddIndexIfNotFound(pt.z);
      }

      int num_x = x_indices_by_lat.GetNumUniqueIndices();
      int num_y = y_indices_by_lon.GetNumUniqueIndices();
      int num_z = z_indices_by_depth.GetNumUniqueIndices();

      index_table.resize(num_z);
      for(int i = 0; i < num_z; ++i)
      {
        index_table[i].resize(num_y);
        for(int j = 0; j < num_y; ++j)
        {
          index_table[i][j].resize(num_x);
        }
      }

      for(std::size_t i = 0; i < _cloud.size(); ++i)
      {
        auto pt = _cloud[i];
        std::size_t x_index = x_indices_by_lat.GetIndex(pt.x).value();
        std::size_t y_index = y_indices_by_lon.GetIndex(pt.y).value();
        std::size_t z_index = z_indices_by_depth.GetIndex(pt.z).value();
        index_table[z_index][y_index][x_index] = i;
      }
    }

    public: std::vector<std::optional<std::size_t>>
      GetInterpolators(pcl::PointXYZ &_pt)
    {
      std::vector<std::optional<std::size_t>> interpolators;
      // PRovides
      auto x_indices = x_indices_by_lat.GetInterpolators(_pt.x);
      auto y_indices = y_indices_by_lon.GetInterpolators(_pt.y);
      auto z_indices = z_indices_by_depth.GetInterpolators(_pt.z);

      /// At the point
      for(auto x_index: x_indices)
      {
        for(auto y_index: y_indices)
        {
          for(auto z_index: z_indices)
          {
            auto index = index_table[z_index][y_index][x_index];
            interpolators.push_back(index);
          }
        }
      }

      return interpolators;
    }
  };
}

#endif