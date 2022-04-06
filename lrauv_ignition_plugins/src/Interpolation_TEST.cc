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
// This tests the 3D case.
/* TODO: Not passing yet. Need to fix _values indexing
TEST(InterpolationTest, TrilinearFallbackToHybridBarylinear3D)
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
  //    0|      *    *  (4 points)
  //     |       q
  //   -1|
  //   -2| *    *       (4 points)
  //
  // A rectangle at z = 0
  cloud.points.push_back(pcl::PointXYZ( 0, 0, 0));
  cloud.points.push_back(pcl::PointXYZ( 1, 0, 0));
  cloud.points.push_back(pcl::PointXYZ( 0, 1, 0));
  cloud.points.push_back(pcl::PointXYZ( 1, 1, 0));
  // Same rectangle at z = -2
  cloud.points.push_back(pcl::PointXYZ( 0, 0, -2));
  cloud.points.push_back(pcl::PointXYZ(-1, 0, -2));
  cloud.points.push_back(pcl::PointXYZ( 0, 1, -2));
  cloud.points.push_back(pcl::PointXYZ(-1, 1, -2));

  // Make a query point that is the general 3D quad case, but not prism
  pcl::PointXYZ queryPt(0.1, 0, -0.3);
  // 1st NN is at this index
  int nnIdx = 0;

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

  // Check neighbors' indices.
  // Degenerate into a 2D triangle.
  // Top-down view of 4 nearest neighbors on 1st z slice, not in a rectangle,
  // therefore the 8 neighbors together won't be in a prism:
  // z = 0:
  //            y
  //            |
  //   1        c    d
  //   0 -------a----b- x
  //            |
  //      -1    0    1
  // z = -2:
  //            y
  //            |
  //   1   h    f
  //   0 --g----e------ x
  //            |
  //      -1    0    1
  EXPECT_EQ(interpInds1[0], 0);  // a
  EXPECT_EQ(interpInds1[1], 1);  // b
  EXPECT_EQ(interpInds1[2], 2);  // c
  EXPECT_EQ(interpInds1[3], 3);  // d

  // 2nd z slice only has 4 points
  EXPECT_EQ(interpInds2[0], 4);  // e
  EXPECT_EQ(interpInds2[1], 6);  // f
  EXPECT_EQ(interpInds2[2], 5);  // g
  EXPECT_EQ(interpInds2[3], 7);  // h

  // Package into variable types for interpolation function
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
  data.push_back(   0.0f);  // a
  data.push_back(   0.0f);  // b
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
  // Result should be between the two z slices
  EXPECT_GT(result, 0);
  EXPECT_LT(result, 1000);
}
*/

/////////////////////////////////////////////////
// Happens when trilinear interpolation assumption of points being corners of
// a prism is not satisfied, falls back to hybrid "barylinear" interpolation,
// i.e. linear interpolation between 3D non-prism quads in two z slices, or the
// degenerative cases of a 2D triangle, or a 1D line segment.
// This tests the 2D case from the 3D overloaded BarycentricInterpolate().
TEST(InterpolationTest, TrilinearFallbackToHybridBarylinear2D)
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
  //             q
  //   0 --*----*----*- x
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

  // Query point q in middle column, slightly +x. Its nearest neighbors are not
  // corners of a prism.
  pcl::PointXYZ queryPt(0.1, 0.3, 0);
  // 1st NN is at this index
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

  // Check neighbors' indices.
  // Degenerate into a 2D triangle.
  // Top-down view of 4 nearest neighbors on 1st z slice, not in a rectangle,
  // therefore the 8 neighbors together won't be in a prism:
  //            y
  //            |
  //   1        c
  //             q
  //   0 --d----a----b- x
  //            |
  //      -1    0    1
  EXPECT_EQ(interpInds1[0], 8);  // a
  EXPECT_EQ(interpInds1[1], 9);  // b
  EXPECT_EQ(interpInds1[2], 1);  // c
  EXPECT_EQ(interpInds1[3], 3);  // d

  // 2nd z slice only has 4 points
  EXPECT_EQ(interpInds2[0], 5);
  EXPECT_EQ(interpInds2[1], 7);
  EXPECT_EQ(interpInds2[2], 4);
  EXPECT_EQ(interpInds2[3], 6);

  // Package into variable types for interpolation function
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
  data.push_back(   0.0f);  // a
  data.push_back(  25.0f);  // b
  data.push_back(  50.0f);  // c
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
  // Result should be among a, b, and c
  EXPECT_GT(result, 0);
  EXPECT_LT(result, 50);
}

/////////////////////////////////////////////////
// Happens when trilinear interpolation assumption of points being corners of
// a prism is not satisfied, falls back to hybrid "barylinear" interpolation,
// i.e. linear interpolation between 3D non-prism quads in two z slices, or the
// degenerative cases of a 2D triangle, or a 1D line segment.
// This tests the 1D case from the 2D overloaded BarycentricInterpolate().
TEST(InterpolationTest, TrilinearFallbackToHybridBarylinear1D)
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
  // Top-down view (each corner point is 2 points overlapped):
  //            y
  //            |
  //   0 ---*-*-*q--*---- x
  //            |
  //       -1   0   1
  // z view:
  //     z
  //    0| * *  *    *
  //   -2| * *  *    *
  //
  // A line at z = 0
  cloud.points.push_back(pcl::PointXYZ(-1, 0, 0));
  cloud.points.push_back(pcl::PointXYZ(-0.5, 0, 0));
  cloud.points.push_back(pcl::PointXYZ( 0, 0, 0));
  cloud.points.push_back(pcl::PointXYZ( 1, 0, 0));
  // Same line at z = -2
  cloud.points.push_back(pcl::PointXYZ(-1, 0, -2));
  cloud.points.push_back(pcl::PointXYZ(-0.5, 0, -2));
  cloud.points.push_back(pcl::PointXYZ( 0, 0, -2));
  cloud.points.push_back(pcl::PointXYZ( 1, 0, -2));

  pcl::PointXYZ queryPt(0.1, 0, 0);
  // 1st NN is at this index
  int nnIdx = 2;

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

  // Check neighbors' indices.
  // Degenerate into a 1D line.
  // Top-down view:
  //            |
  //   0 ---d-b-aq--c---- x
  //            |
  //       -1   0   1
  EXPECT_EQ(interpInds1[0], 2);  // a
  EXPECT_EQ(interpInds1[1], 1);  // b
  EXPECT_EQ(interpInds1[2], 3);  // c
  EXPECT_EQ(interpInds1[3], 0);  // d

  // 2nd z slice only has 4 points
  EXPECT_EQ(interpInds2[0], 6);
  EXPECT_EQ(interpInds2[1], 5);
  EXPECT_EQ(interpInds2[2], 7);
  EXPECT_EQ(interpInds2[3], 4);

  // Package into variable types for interpolation function
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
  data.push_back(  25.0f);  // b
  data.push_back( 100.0f);  // c
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
  // Result should be between closest points on the line to q
  EXPECT_GT(result, 50);
  EXPECT_LT(result, 100);
}
}
