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

#include <gtest/gtest.h>

#include "Interpolation.hh"

namespace tethys
{

TEST(InterpolationTest, SetAndGetMethod)
{
  // Constructor using default arg
  Interpolation interp_dft;
  EXPECT_EQ(interp_dft.Method(), TRILINEAR);

  // Constructor using default arg explicitly
  Interpolation interp1(TRILINEAR);
  EXPECT_EQ(interp1.Method(), TRILINEAR);

  interp1.SetMethod(BARYCENTRIC);
  EXPECT_EQ(interp1.Method(), BARYCENTRIC);

  // Constructor using alternative arg
  Interpolation interp2(BARYCENTRIC);
  EXPECT_EQ(interp2.Method(), BARYCENTRIC);

  interp2.SetMethod(TRILINEAR);
  EXPECT_EQ(interp2.Method(), TRILINEAR);
}

TEST(InterpolationTest, EightPointPrismTrilinear)
{
  Interpolation interp(TRILINEAR);
  EXPECT_EQ(interp.Method(), TRILINEAR);

  float spatialRes = 0.1f;
  int k = 4;

  // Inputs
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ queryPt(4, -5, 0);

  // Populate input cloud. 8-point prism
  // Cartesian points corresponding to these (lat, long) in ../data/*.csv file:
  //   -0.00003,0.00003,0,0,0,300,-1,0
  //   -0.00006,0.00003,0,0,0,300,-1,0
  //   -0.00003,0.00006,0,0,0,300,-1,0
  //   -0.00006,0.00006,0,0,0,300,-1,0
  //   -0.00003,0.00003,2,0,0,100,-1,0
  //   -0.00006,0.00003,2,0,0,100,-1,0
  //   -0.00003,0.00006,2,0,0,100,-1,0
  //   -0.00006,0.00006,2,0,0,100,-1,0
  //   1,1,100,0,0,NaN,0,0
  // Corresponding debug output from FindTrilinearInterpolators():
  //   [Dbg] 9 points in full cloud
  //   [Dbg] 1st nn (3.33958, -6.63446, -0), idx 1, dist 1.76284, z slice 4 points
  //   [Dbg] Found 4 neighbors.
  //   [Dbg] Neighbor at (3.34, -6.634, -0), distance 1.76284 m
  //   [Dbg] Neighbor at (3.34, -3.317, -0), distance 1.80773 m
  //   [Dbg] Neighbor at (6.679, -6.634, -0), distance 3.13837 m
  //   [Dbg] Neighbor at (6.679, -3.317, -0), distance 3.16381 m
  //   [Dbg] Excluding 1st nn z slice. Remaining cloud has 5 points
  //   [Dbg] Found 1 neighbors.
  //   [Dbg] Neighbor at (3.34, -6.634, -2), distance 2.66601 m
  //   [Dbg] 2nd nn (3.33958, -6.63446, -0), idx 1, dist 2.66601, z slice 4 points
  //   [Dbg] Found 4 neighbors.
  //   [Dbg] Neighbor at (3.34, -6.634, -2), distance 2.66601 m
  //   [Dbg] Neighbor at (3.34, -3.317, -2), distance 2.6959 m
  //   [Dbg] Neighbor at (6.679, -6.634, -2), distance 3.72148 m
  //   [Dbg] Neighbor at (6.679, -3.317, -2), distance 3.74295 m
  // A rectangle at z = 0
  cloud.points.push_back(pcl::PointXYZ(3.33958, -3.31723, 0));
  cloud.points.push_back(pcl::PointXYZ(3.33958, -6.63446, 0));  // 1st NN
  cloud.points.push_back(pcl::PointXYZ(6.679, -3.31723, 0));
  cloud.points.push_back(pcl::PointXYZ(6.679, -6.63446, 0));
  // Same rectangle at z = -2
  cloud.points.push_back(pcl::PointXYZ(3.33958, -3.31723, -2));
  cloud.points.push_back(pcl::PointXYZ(3.33958, -6.63446, -2));  // 2nd NN
  cloud.points.push_back(pcl::PointXYZ(6.679, -3.31723, -2));
  cloud.points.push_back(pcl::PointXYZ(6.679, -6.63446, -2));
  // An outlier
  cloud.points.push_back(pcl::PointXYZ(100, 100, -100));
  // 1st NN is at index [1]
  int nnIdx = 1;

  // Return values from trilinear search
  std::vector<int> interpInds1, interpInds2;
  std::vector<pcl::PointXYZ> interps1, interps2;

  // Search function. Search neighbors to do trilinear interpolation among
  interp.FindTrilinearInterpolators(cloud, queryPt, nnIdx, spatialRes,
    interpInds1, interps1, interpInds2, interps2, k);

  // Check neighbors found by trilinear search
  EXPECT_EQ(interpInds1.size(), k);
  EXPECT_EQ(interps1.size(), k);
  EXPECT_EQ(interpInds2.size(), k);
  EXPECT_EQ(interps2.size(), k);

  // Indices of points in 1st z slice found
  EXPECT_EQ(interpInds1[0], 1);
  EXPECT_EQ(interpInds1[1], 0);
  EXPECT_EQ(interpInds1[2], 3);
  EXPECT_EQ(interpInds1[3], 2);

  // Indices of points in 2nd z slice found
  EXPECT_EQ(interpInds2[0], 5);
  EXPECT_EQ(interpInds2[1], 4);
  EXPECT_EQ(interpInds2[2], 7);
  EXPECT_EQ(interpInds2[3], 6);
}

TEST(InterpolationTest, EightPointPrismBarycentric)
{
  Interpolation interp(BARYCENTRIC);
  EXPECT_EQ(interp.Method(), BARYCENTRIC);

  float spatialRes = 0.1f;
  int k = 4;
}
}
