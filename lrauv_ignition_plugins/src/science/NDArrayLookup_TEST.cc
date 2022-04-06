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
#include "NDArrayLookup.hh"

#include <gtest/gtest.h>
using namespace tethys;


TEST(NDArrayLookup, CheckInterpolationExact)
{
  // This tests query performance of 54000 points in a 3D grid.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const double stride_x = 1, stride_y = 5, stride_z = 10;
  for(double x = 0; x < 300; x += stride_x)
  {
    for(double y = 0; y < 300; y += stride_y)
    {
      for(double z = 0; z < 300; z += stride_z)
      {
        cloud.push_back(pcl::PointXYZ(x, y, z));
      }
    }
  }
  VolumetricScalarField scalarIndex(cloud);

  for(std::size_t i = 0; i < cloud.size(); ++i)
  {
    auto val = scalarIndex.GetInterpolators(cloud[i]);
    ASSERT_EQ(val.size(), 1);
    ASSERT_EQ(val[0], i);
  }
}

TEST(NDArrayLookup, CheckInterpolationBoxEightPoints)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0, 0, 0));
  cloud.push_back(pcl::PointXYZ(0, 0, 1));
  cloud.push_back(pcl::PointXYZ(0, 1, 0));
  cloud.push_back(pcl::PointXYZ(0, 1, 1));
  cloud.push_back(pcl::PointXYZ(1, 0, 0));
  cloud.push_back(pcl::PointXYZ(1, 0, 1));
  cloud.push_back(pcl::PointXYZ(1, 1, 0));
  cloud.push_back(pcl::PointXYZ(1, 1, 1));
  /*for(double x = 0; x < 300; x += stride_x)
  {
    for(double y = 0; y < 300; y += stride_y)
    {
      for(double z = 0; z < 300; z += stride_z)
      {
        cloud.push_back(pcl::PointXYZ(x, y, z));
      }
    }
  }

  VolumetricScalarField scalarIndex(cloud);

  for(std::size_t i = 0; i < cloud.size(); ++i)
  {
    auto val = scalarIndex.GetInterpolators(cloud[i]);
    ASSERT_EQ(val.size(), 1);
    ASSERT_EQ(val[0], i);
  }*/
}