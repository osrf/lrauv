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

#include <ignition/common/Console.hh>

#include "Interpolation.hh"

namespace tethys
{

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
TEST(InterpolationTest, TrilinearSearchEightPointPrism)
{
  Interpolation interp(TRILINEAR);
  EXPECT_EQ(interp.Method(), TRILINEAR);

  float spatialRes = 0.1f;
  int k = 4;

  // Inputs to search function
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Populate input cloud. 8-point prism
  // A rectangle at z = 0
  cloud.points.push_back(pcl::PointXYZ(0, 0, 0));
  cloud.points.push_back(pcl::PointXYZ(1, 0, 0));
  cloud.points.push_back(pcl::PointXYZ(0, 1, 0));
  cloud.points.push_back(pcl::PointXYZ(1, 1, 0));
  // Same rectangle at z = -2
  cloud.points.push_back(pcl::PointXYZ(0, 0, -2));
  cloud.points.push_back(pcl::PointXYZ(1, 0, -2));
  cloud.points.push_back(pcl::PointXYZ(0, 1, -2));
  cloud.points.push_back(pcl::PointXYZ(1, 1, -2));
  // An outlier
  cloud.points.push_back(pcl::PointXYZ(100, 100, -100));

  //////////////////////////////////
  // Test normal case, query point inside a prism cell

  // Query point is almost centered in upper z slice, but slightly toward
  // point [1]
  pcl::PointXYZ queryPt(0.6, 0.3, 0);
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

  //////////////////////////////////
  // Test edge case: query point is outside the prism. Happens when query point
  // is outside the boundaries of regions where data are available.

  // Move query point outside and above prism. (Nearest neighbor remains same)
  pcl::PointXYZ queryPtAbove(queryPt.x, queryPt.y, 1);
  interp.FindTrilinearInterpolators(cloud, queryPtAbove, nnIdx, spatialRes,
    interpInds1, interps1, interpInds2, interps2, k);
  // Check that it still finds two z slices, even though the point is not
  // between the slices (expected behavior, can be improved in algorithm).
  EXPECT_EQ(interpInds1.size(), k);
  EXPECT_EQ(interps1.size(), k);
  EXPECT_EQ(interpInds2.size(), k);
  EXPECT_EQ(interps2.size(), k);

  //////////////////////////////////
  // Test invalid cases

  // Move query outside prism, to overlap with outlier
  pcl::PointXYZ queryPtOutlier(100, 100, -100);
  nnIdx = 8;
  interp.FindTrilinearInterpolators(cloud, queryPtOutlier, nnIdx, spatialRes,
    interpInds1, interps1, interpInds2, interps2, k);
  // Check that it cannot find 2 z slices for query point
  EXPECT_EQ(interpInds1.size(), 0);

  // Invalid index for nearest neighbor
  interp.FindTrilinearInterpolators(cloud, queryPt, -1, spatialRes,
    interpInds1, interps1, interpInds2, interps2, k);
  EXPECT_EQ(interpInds1.size(), 0);

  // Remove a point in a z slice
  pcl::PointCloud<pcl::PointXYZ> cloudShort = cloud;
  cloudShort.points.erase(cloudShort.points.begin() + 0);
  interp.FindTrilinearInterpolators(cloudShort, queryPt, 0, spatialRes,
    interpInds1, interps1, interpInds2, interps2, k);
  // Check that it cannot find 2 z slices for query point
  EXPECT_EQ(interpInds1.size(), 0);

  // Empty cloud
  pcl::PointCloud<pcl::PointXYZ> emptyCloud;
  interp.FindTrilinearInterpolators(emptyCloud, queryPt, nnIdx, spatialRes,
    interpInds1, interps1, interpInds2, interps2, k);
  // Check the function clears return values
  EXPECT_EQ(interpInds1.size(), 0);
  EXPECT_EQ(interpInds2.size(), 0);
  EXPECT_EQ(interps1.size(), 0);
  EXPECT_EQ(interps2.size(), 0);

  // TODO Add case where point is to left or right of prism, outside prism, but
  // is in between z slices
}

/////////////////////////////////////////////////
TEST(InterpolationTest, TrilinearInterpolationInEightPointPrism)
{
  Interpolation interp(TRILINEAR);
  EXPECT_EQ(interp.Method(), TRILINEAR);

  // Inputs to interpolation function

  // One point per row
  Eigen::MatrixXf interpsEigen(8, 3);
  interpsEigen <<
    // A rectangle at z = 0
    0, 0, 0,
    1, 0, 0,
    0, 1, 0,
    1, 1, 0,
    // Same rectangle at z = -2
    0, 0, -2,
    1, 0, -2,
    0, 1, -2,
    1, 1, -2;

  // Query point is almost centered, lying in upper z slice, but slightly toward
  // point [1]
  Eigen::Vector3f queryPtEigen(0.6, 0.3, 0);

  // Data to test interpolation between two z slices
  std::vector<float> data;
  data.push_back(0.0f);
  data.push_back(0.0f);
  data.push_back(0.0f);
  data.push_back(0.0f);
  data.push_back(200.0f);
  data.push_back(200.0f);
  data.push_back(200.0f);
  data.push_back(200.0f);

  // Interpolation function
  float result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  // Query point at z = 0 should take on average value in z = 0 slice
  EXPECT_EQ(result, 0);

  // Move query to z = -2, on z slice
  queryPtEigen[2] = -2;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  // Query point at z = -2 should take on average value in z = -2 slice
  EXPECT_EQ(result, 200);

  // Move query to z = -1, midpoint between z slices
  queryPtEigen[2] = -1;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  // Query point at z = -1 should take on average value of z = 0 and z = -2
  // slices
  EXPECT_EQ(result, 100);

  // Move query to z = -0.5, a fraction between z slices
  queryPtEigen[2] = -0.5;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  EXPECT_NEAR(result, 50, 1);

  // Move query to z = -1.3, a fraction between z slices
  queryPtEigen[2] = -1.3;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  EXPECT_NEAR(result, 130, 1);

  // Data to test query point overlapping with data point
  std::vector<float> dataUnique;
  dataUnique.push_back(100.0f);
  dataUnique.push_back(200.0f);
  dataUnique.push_back(300.0f);
  dataUnique.push_back(400.0f);
  dataUnique.push_back(500.0f);
  dataUnique.push_back(600.0f);
  dataUnique.push_back(700.0f);
  dataUnique.push_back(800.0f);

  // Move query to overlap exactly with each data point
  for (int i = 0; i < interpsEigen.rows(); ++i)
  {
    queryPtEigen = interpsEigen.row(i);
    result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
    // Query point should take on value exactly at the point it overlaps with
    EXPECT_EQ(result, data[i]);
  }

  // Test invalid input for which function returns NaN
  interpsEigen.resize(9, 3);
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  EXPECT_TRUE(std::isnan(result));
}

/////////////////////////////////////////////////
// Happens when trilinear interpolation assumption of points being corners of
// a prism is not satisfied, falls back to hybrid "barylinear" interpolation,
// i.e. linear interpolation between 3D non-prism quads in two z slices, or the
// degenerative cases of a 2D triangle, or a 1D line segment.
TEST(InterpolationTest, TrilinearFallbackToHybridBarylinear)
{
  ignition::common::Console::SetVerbosity(4);

  Interpolation interp(TRILINEAR);
  EXPECT_EQ(interp.Method(), TRILINEAR);

  interp.SetDebug(true);
  interp.SetDebugMath(false);

  // Inputs to search function
  float spatialRes = 0.1f;
  int k = 4;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Populate input cloud
  // Top-down view:
  //            y
  //            |
  //   1   *    *    *
  //   0 --*----*q---*- x
  //            |
  //      -1    0    1
  // z view:
  //     z
  //    0| *    *    *
  //   -2| *         *
  //
  // A rectangle at z = 0
  cloud.points.push_back(pcl::PointXYZ(-1, 0, 0));
  cloud.points.push_back(pcl::PointXYZ( 1, 0, 0));
  cloud.points.push_back(pcl::PointXYZ(-1, 1, 0));
  cloud.points.push_back(pcl::PointXYZ( 1, 1, 0));
  // Same rectangle at z = -2
  cloud.points.push_back(pcl::PointXYZ(-1, 0, -2));
  cloud.points.push_back(pcl::PointXYZ( 1, 0, -2));
  cloud.points.push_back(pcl::PointXYZ(-1, 1, -2));
  cloud.points.push_back(pcl::PointXYZ( 1, 1, -2));
  // A middle column at z = 0 only, between two sides of the rectangle
  cloud.points.push_back(pcl::PointXYZ(0, 0, 0));
  cloud.points.push_back(pcl::PointXYZ(0, 1, 0));

  // Query point in middle column, slightly +x. Its nearest neighbors are not
  // corners of a prism.
  pcl::PointXYZ queryPt(0.1, 0, 0);
  // 1st NN is at index [8]
  int nnIdx = 8;

  // Return values from trilinear search
  std::vector<int> interpInds1, interpInds2;
  std::vector<pcl::PointXYZ> interps1, interps2;

  // Search function. Search neighbors to do trilinear interpolation among
  interp.FindTrilinearInterpolators(cloud, queryPt, nnIdx, spatialRes,
    interpInds1, interps1, interpInds2, interps2, k);

  // Check that it finds 2 z slices
  EXPECT_EQ(interpInds1.size(), 4);
  EXPECT_EQ(interpInds2.size(), 4);
  EXPECT_EQ(interps1.size(), 4);
  EXPECT_EQ(interps2.size(), 4);

  // TODO Make a query point that is the general 3D quad case, but not prism

  // Check neighbors' indices.
  // Degenerate into a 2D triangle.
  // Top-down view of 4 nearest neighbors on 1st z slice, not in a rectangle,
  // therefore the 8 neighbors together won't be in a prism:
  //            y
  //            |
  //   1        c
  //   0 --d----aq---b- x
  //            |
  //      -1    0    1
  EXPECT_EQ(interpInds1[0], 8);  // a
  EXPECT_EQ(interpInds1[1], 1);  // b
  EXPECT_EQ(interpInds1[2], 9);  // c
  EXPECT_EQ(interpInds1[3], 0);  // d

  // 2nd z slice only has 4 points
  EXPECT_EQ(interpInds2[0], 5);
  EXPECT_EQ(interpInds2[1], 4);
  EXPECT_EQ(interpInds2[2], 7);
  EXPECT_EQ(interpInds2[3], 6);

  Eigen::Vector3f queryPtEigen(queryPt.x, queryPt.y, queryPt.z);
  Eigen::MatrixXf interpsEigen(8, 3);
  for (int i = 0; i < interpInds1.size(); ++i)
  {
    interpsEigen.row(i) = Eigen::Vector3f(
      cloud.points[interpInds1[i]].x,
      cloud.points[interpInds1[i]].y,
      cloud.points[interpInds1[i]].z);
  }
  for (int i = 0; i < interpInds2.size(); ++i)
  {
    interpsEigen.row(4 + i) = Eigen::Vector3f(
      cloud.points[interpInds2[i]].x,
      cloud.points[interpInds2[i]].y,
      cloud.points[interpInds2[i]].z);
  }

  // Data to test interpolation between two z slices
  std::vector<float> data;
  data.push_back(  50.0f);  // a
  data.push_back( 100.0f);  // b
  data.push_back(   0.0f);  // c
  data.push_back(   0.0f);  // d
  // Set values in 2nd z slice very different from those on 1st z slice.
  // Since query point is entirely on 1st z slice, it should be independent
  // of values on 2nd z slice.
  data.push_back(1000.0f);
  data.push_back(1000.0f);
  data.push_back(1000.0f);
  data.push_back(1000.0f);

  // Interpolation function
  float result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  // Result should be between a and b
  EXPECT_GT(result, 50);
  EXPECT_LT(result, 100);

  // TODO Make a query point that is the degenerate 1D line case
}
}
