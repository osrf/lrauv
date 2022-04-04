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

#ifndef __LRAUV_INTERPOLATION_METHODS_HH__
#define __LRAUV_INTERPOLATION_METHODS_HH__

#include <ignition/common/Console.hh>
#include <Eigen/Eigen>

namespace tethys {
/////////////////////////////////////////////////
float InterpolationPrivate::TrilinearInterpolate(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{

  // Sanity check: Must have 8 points, 4 above, 4 below.
  if (_xyzs.rows() != 8)
  {
    ignerr << "Size of interpolators invalid (" << _xyzs.rows() << "). "
      << "Need 8 points in a rectangular prism. "
      << "Cannot perform trilinear interpolation." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }

  // A rectangular prism can be represented by two pairs of 3D coordinates,
  // (x0, y0, z0) and (x1, y1, z1), which are diagonal vertices on the prism.
  // Extract 2 diagonal vertices to represent the rectangular prism, assuming
  // the points are corners of a prism (assumption checked after extracting).
  // Corner of minimum x y z
  auto v000 = Eigen::Vector3f(
    _xyzs.col(0).minCoeff(),
    _xyzs.col(1).minCoeff(),
    _xyzs.col(2).minCoeff());
  // Corner of maximum x y z
  auto v111 = Eigen::Vector3f(
    _xyzs.col(0).maxCoeff(),
    _xyzs.col(1).maxCoeff(),
    _xyzs.col(2).maxCoeff());
  // 6 remaining vertices of prism
  Eigen::Vector3f v001, v010, v011, v100, v101, v110;

  igndbg << "Trilinear interpolation min vert v000: "
     << v000(0) << ", " << v000(1) << ", " << v000(2) << std::endl;
  igndbg << "Trilinear interpolation max vert v111: "
     << v111(0) << ", " << v111(1) << ", " << v111(2) << std::endl;

  // Data values at the vertices
  // Define explicitly for readability in interpolation equations later
  float d000, d001, d010, d011, d100, d101, d110, d111;

  // Tolerance below which to consider two values equal
  const double TOLERANCE = 1e-6;

  // Find ordering of corners in prism, assign data to ordered corners.
  // Each vertex, if really on the corner of a rectangular prism, must share
  // at least one component of its xyz coordinates with v000, and the other
  // components are shared with v111.
  // The actual vertices are v000, v001, v010, v011, v100, v101, v110, v111.
  // Example: v100 = (v111_x, v000_y, v000_z). Similar for others.
  for (int r = 0; r < _xyzs.rows(); ++r)
  {
    /// 000 or [0]
    if (fabs(_xyzs(r, 0) - v000(0)) <= TOLERANCE &&
        fabs(_xyzs(r, 1) - v000(1)) <= TOLERANCE &&
        fabs(_xyzs(r, 2) - v000(2)) <= TOLERANCE)
    {
      d000 = _values[r];
    }
    // 001 or [1]
    else if (fabs(_xyzs(r, 0) - v000(0)) <= TOLERANCE &&
             fabs(_xyzs(r, 1) - v000(1)) <= TOLERANCE &&
             fabs(_xyzs(r, 2) - v111(2)) <= TOLERANCE)
    {
      v001 = _xyzs.row(r);
      d001 = _values[r];
    }
    // 010 or [2]
    else if (fabs(_xyzs(r, 0) - v000(0)) <= TOLERANCE &&
             fabs(_xyzs(r, 1) - v111(1)) <= TOLERANCE &&
             fabs(_xyzs(r, 2) - v000(2)) <= TOLERANCE)
    {
      v010 = _xyzs.row(r);
      d010 = _values[r];
    }
    // 011 or [3]
    else if (fabs(_xyzs(r, 0) - v000(0)) <= TOLERANCE &&
             fabs(_xyzs(r, 1) - v111(1)) <= TOLERANCE &&
             fabs(_xyzs(r, 2) - v111(2)) <= TOLERANCE)
    {
      v011 = _xyzs.row(r);
      d011 = _values[r];
    }
    // 100 or [4]
    else if (fabs(_xyzs(r, 0) - v111(0)) <= TOLERANCE &&
             fabs(_xyzs(r, 1) - v000(1)) <= TOLERANCE &&
             fabs(_xyzs(r, 2) - v000(2)) <= TOLERANCE)
    {
      v100 = _xyzs.row(r);
      d100 = _values[r];
    }
    // 101 or [5]
    else if (fabs(_xyzs(r, 0) - v111(0)) <= TOLERANCE &&
             fabs(_xyzs(r, 1) - v000(1)) <= TOLERANCE &&
             fabs(_xyzs(r, 2) - v111(2)) <= TOLERANCE)
    {
      v101 = _xyzs.row(r);
      d101 = _values[r];
    }
    // 110 or [6]
    else if (fabs(_xyzs(r, 0) - v111(0)) <= TOLERANCE &&
             fabs(_xyzs(r, 1) - v111(1)) <= TOLERANCE &&
             fabs(_xyzs(r, 2) - v000(2)) <= TOLERANCE)
    {
      v110 = _xyzs.row(r);
      d110 = _values[r];
    }
    // 111 or [7]
    else if (fabs(_xyzs(r, 0) - v111(0)) <= TOLERANCE &&
             fabs(_xyzs(r, 1) - v111(1)) <= TOLERANCE &&
             fabs(_xyzs(r, 2) - v111(2)) <= TOLERANCE)
    {
      d111 = _values[r];
    }
    else
    {
      // Enforce that the points are vertices of a rectangular prism.
      // Otherwise does not satisfy trilinear interpolation requirements.
      ignwarn << "Trilinear interpolation: "
        << "Suspect 8 input points not on prism. Vertex " << r << " ("
        << std::round(_xyzs(r, 0) * 1000.0) / 1000.0 << ", "
        << std::round(_xyzs(r, 1) * 1000.0) / 1000.0 << ", "
        << std::round(_xyzs(r, 2) * 1000.0) / 1000.0
        << ") not within tolerance (" << TOLERANCE
        << ") of any of 8 vertices of rectangular prism. "
        << "Using hybrid bary-linear interpolation instead." << std::endl;
      return BaryLinearInterpolate(_p, _xyzs,_values);
    }
  }

  ///if (this->debugGeneral)
  ///  igndbg << "Trilinear interpolation, starting with 8 points: "
  ///    << d000 << ", " << d001 << ", " << d010 << ", " << d011 << ", "
  ///    << d100 << ", " << d101 << ", " << d110 << ", " << d111
  ///    << std::endl;

  // Trilinear interpolation using 8 corners of a rectangular prism
  // Implements https://en.wikipedia.org/wiki/Trilinear_interpolation

  // Ratio describing where the target point is within the cube
  float dx = (_p(0) - v000(0)) / (v111(0) - v000(0));
  float dy = (_p(1) - v000(1)) / (v111(1) - v000(1));
  float dz = (_p(2) - v000(2)) / (v111(2) - v000(2));

  // By definition of trilinear interpolation, the order you interpolate among
  // xyz does not matter. They will give the same result.
  // Interpolate along x, to find where target point is wrt each pair of x's.
  // For 8 points, there are 4 pairs of x's.
  float d00 = d000 * (1 - dx) + d100 * dx;
  float d01 = d001 * (1 - dx) + d101 * dx;
  float d10 = d010 * (1 - dx) + d110 * dx;
  float d11 = d011 * (1 - dx) + d111 * dx;

  if (this->debugGeneral)
    igndbg << "Trilinear interpolation, 4 intermediate results from 8 points: "
      << d00 << ", " << d01 << ", " << d10 << ", " << d11 << std::endl;

  // Interpolate along y
  // For 4 interpolated points on x above, there are 2 pairs of y's
  float d0 = d00 * (1 - dy) + d10 * dy;
  float d1 = d01 * (1 - dy) + d11 * dy;

  if (this->debugGeneral)
    igndbg << "Trilinear interpolation, 2 intermediate results from 4 points: "
      << d0 << ", " << d1 << std::endl;

  // Interpolate along z
  // For 2 interpolated points on y above, there is only 1 z.
  // This arrives at the interpolated value at the target xyz position.
  float d = d0 * (1 - dz) + d1 * dz;

  if (this->debugGeneral)
    igndbg << "Trilinear interpolation result: " << d << std::endl;

  return d;
}

/////////////////////////////////////////////////
float BaryLinearInterpolate(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{
  // Drop z dimension, since we are assuming triangles are on z slices
  Eigen::Vector2f p2d;
  p2d << _p(0), _p(1);

  if (_xyzs.rows() != 8 || _values.size() != 8)
  {
    ignerr << "BaryLinearInterpolate(): Unexpected number of neighbors ("
      << _xyzs.rows() << " and " << _values.size() << "). "
      << "Aborting interpolation." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Step 1
  // Interpolate among the 4 points in each z slice, to get 2 resulting points,
  // at (x, y, z1) and (x, y, z2), where (x, y, z) is the query point, and z1
  // and z2 are the z's of the slices.
  // Assume [0]-[3] is first slice, [4]-[7] is 2nd slice.
  // 4 x 2. Drop z dimension
  Eigen::Matrix<float, 4, 2> xysSlice1 = _xyzs.block(0, 0, 4, 2);
  Eigen::Matrix<float, 4, 2> xysSlice2 = _xyzs.block(4, 0, 4, 2);
  float valueSlice2 = BarycentricInterpolate(p2d, xysSlice2, _values);
  float valueSlice1 = BarycentricInterpolate(p2d, xysSlice1, _values);
  ///if (this->debugGeneral)
  ///  igndbg << "Hybrid bary-linear interpolation, "
  ///    << "2 barycentric interpolations on 2 z slices resulted in: "
  ///    << valueSlice1 << " and " << valueSlice2 << std::endl;

  // Assume [0]-[3] is first slice, [4]-[7] is 2nd slice.
  float z1 = _xyzs(0, 2);
  float z2 = _xyzs(4, 2);
  float z = _p(2);

  // Step 2
  // Linear interpolation between the 2 intermediate points, (x, y, z1) and
  // (x, y, z2). Interpolate linearly along z to get value at query point
  // (x, y, z).
  float dz1 = fabs(z1 - z);
  float dz2 = fabs(z2 - z);
  float dz1Weight = dz1 / (dz1 + dz2);
  float dz2Weight = dz2 / (dz1 + dz2);
  float result = dz1Weight * valueSlice1 + dz2Weight * valueSlice2;
  //if (this->debugGeneral)
  //  igndbg << "Hybrid bary-linear interpolation, "
  //    << "linear interpolation of 2 points on two z slices: " << result
  //    << std::endl;

  //if (this->debugGeneral)
  //{
  //  igndbg << "Hybrid bary-linear interpolation of " << _values.size()
  //    << " values:" << std::endl;
  //  for (int i = 0; i < _values.size(); ++i)
  //    igndbg << _values[i] << std::endl;
  //  igndbg << "resulted in " << result << std::endl;
  //}

  return result;
}
}