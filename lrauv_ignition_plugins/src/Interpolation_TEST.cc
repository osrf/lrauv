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

TEST(InterpolationTest, TrilinearSearchWithinEightPointPrism)
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

  // Query point is almost centered in upper slice, but slightly toward
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
}

TEST(InterpolationTest, TrilinearInterpolationWithinEightPointPrism)
{
  Interpolation interp(TRILINEAR);
  EXPECT_EQ(interp.Method(), TRILINEAR);

  // Inputs to interpolation function

  // Query point at z = 0
  Eigen::Vector3f queryPtEigen(0.6, 0.3, 0);

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

  // Move query to z = -2
  queryPtEigen[2] = -2;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  // Query point at z = -2 should take on average value in z = -2 slice
  EXPECT_EQ(result, 200);

  // Move query to z = -1
  queryPtEigen[2] = -1;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  // Query point at z = -1 should take on average value of z = 0 and z = -2
  // slices
  EXPECT_EQ(result, 100);

  // Move query to z = -0.5
  queryPtEigen[2] = -0.5;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  EXPECT_NEAR(result, 50, 1);

  // Move query to z = -1.3
  queryPtEigen[2] = -1.3;
  result = interp.InterpolateData(queryPtEigen, interpsEigen, data);
  EXPECT_NEAR(result, 130, 1);

  // Data to test when query point overlaps with data point
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
}
