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

#ifndef TETHYS_INTERPOLATION_HH_
#define TETHYS_INTERPOLATION_HH_

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "Interpolation.hh"

namespace tethys
{
class InterpolationPrivate;

/// \brief Algorithms for data interpolation
class Interpolation
{
  public: enum interpolation
  {
    TRILINEAR,
    BARYCENTRIC
  };

  public: Interpolation();

  /// \brief Interpolate among existing science data to output an estimated
  /// reading at the current sensor location.
  /// \param[in] _p Position to interpolate for
  /// \param[in] _xyzs XYZ coordinates of existing data locations
  /// \param[in] _values Data values at the locations
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float InterpolateData(
    const Eigen::Vector3f &_p,
    const Eigen::MatrixXf &_xyzs,
    const std::vector<float> &_values);

  /// \brief Find XYZ locations of points in the two closest z slices to
  /// interpolate among.
  /// \param[in] _cloud Spatial locations of existing data
  /// \param[in] _pt Location in space to interpolate for
  /// \param[in] _inds Indices of nearest neighbors to _pt, used to look for
  /// first z slice.
  /// \param[in] _sqrDists Distances of nearest neighbors to _pt
  /// \param[in] _spatialRes Spatial resolution to create an octree for the
  /// search
  /// \param[out] _interpolatorInds2 Indices of points in fist z slice
  /// \param[out] _interpolators1 XYZ points on a z slice to interpolate among
  /// \param[out] _interpolatorInds1 Indices of points in second z slice
  /// \param[out] _interpolators2 XYZ points on a second z slice to interpolate
  /// among
  /// \param[in] _k Number of nearest neighbors. Default to 4, for trilinear
  /// interpolation between two z slices of 4 points per slice.
  public: void FindTrilinearInterpolators(
    pcl::PointCloud<pcl::PointXYZ> &_cloud,
    pcl::PointXYZ &_pt,
    std::vector<int> &_inds,
    std::vector<float> &_sqrDists,
    float _spatialRes,
    std::vector<int> &_interpolatorInds1,
    std::vector<pcl::PointXYZ> &_interpolators1,
    std::vector<int> &_interpolatorInds2,
    std::vector<pcl::PointXYZ> &_interpolators2,
    int _k=4);

  /// \brief Return debug flag
  /// \return Whether debug flag is on
  public: bool debug();

  /// \brief True to use trilinear interpolation, false to use barycentric
  /// interpolation
  public: int INTERPOLATION_METHOD = TRILINEAR;

  /// \brief Private data pointer
  private: std::unique_ptr<InterpolationPrivate> dataPtr;
};

class InterpolationPrivate
{
  //////////////////////////////////
  // Trilinear interpolation

  /// \brief Comparison function for std::set_difference().
  /// Comparison between points is arbitrary. This function is only used for
  /// set operations, not for literal sorting.
  public: bool comparePclPoints(
    const pcl::PointXYZ &a,
    const pcl::PointXYZ &b);

  /// \brief Create a z slice from a point cloud. Slice contains points sharing
  /// the same given z value.
  /// \param[in] _depth Target z value
  /// \param[in] _cloud Cloud from which to create the z slice
  /// \param[out] _zSlice Points in the new point cloud slice at _depth
  /// \param[out] _zSliceInds Indices of points in _zSlice in the original
  /// point cloud _points
  /// \param[in] _invert Invert the filter. Keep everything except the z slice.
  public: void CreateDepthSlice(
    float _depth,
    pcl::PointCloud<pcl::PointXYZ> &_cloud,
    pcl::PointCloud<pcl::PointXYZ> &_zSlice,
    std::vector<int> &_zSliceInds,
    bool _invert=false);

  /// \brief Create an octree from a point cloud, and search for _k nearest
  /// neighbors.
  /// \param[in] _searchPt Location in space to search from
  /// \param[in] _cloud Point cloud to search in
  /// \param[in] _spatialRes Spatial resolution of the octree
  /// \param[out] _nbrInds Result of octree search, indices of points.
  /// \param[out] _nbrSqrDists Result of octree search, distances.
  /// \param[out] _nbrs XYZ of the k nearest neighbors found.
  /// \param[in] _k Number of nearest neighbors. Default to 4, for trilinear
  /// interpolation between two z slices of 4 points per slice.
  public: void CreateAndSearchOctree(
    pcl::PointXYZ &_searchPt,
    pcl::PointCloud<pcl::PointXYZ> &_cloud,
    float _spatialRes,
    std::vector<int> &_nbrInds,
    std::vector<float> &_nbrSqrDists,
    std::vector<pcl::PointXYZ> &_nbrs,
    int _k=4);

  /// \brief Trilinear interpolation for a point inside a prism, given 8
  /// verticies of the prism. Suitable for data laid out in a rectangular grid.
  /// Otherwise, use a different interpolation method, e.g. barycentric.
  /// If the points must be on different z slices, try the hybrid
  /// BaryLinearInterpolate().
  /// \param[in] _p Position to interpolate for
  /// \param[in] _xyzs XYZ coordinates of 8 vertices of a prism
  /// \param[in] _values Data values at the 8 vertices
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float TrilinearInterpolate(
    const Eigen::Vector3f &_p,
    const Eigen::MatrixXf &_xyzs,
    const std::vector<float> &_values);

  //////////////////////////////////
  // Barycentric interpolation

  /// \brief Barycentric interpolation in 3D, given 4 points on any arbitrary
  /// tetrahedra.
  /// \param[in] _p Position within the tetrahedra to interpolate for
  /// \param[in] _xyzs n x 3. XYZ coordinates of 4 vertices of a tetrahedra
  /// \param[in] _values Data values at the 4 vertices
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float BarycentricInterpolate(
    const Eigen::Vector3f &_p,
    const Eigen::MatrixXf &_xyzs,
    const std::vector<float> &_values);

  /// \brief Barycentric interpolation in 2D, given 4 points on a plane. Finds
  /// 3 points on a triangle within which the query point lies, then
  /// interpolates using them.
  /// \param[in] _p Position within the triangle to interpolate for
  /// \param[in] _xys n x 2. XY coordinates of 3 vertices of a triangle
  /// \param[in] _values Data values at the 3 vertices
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float BarycentricInterpolate(
    const Eigen::Vector2f &_p,
    const Eigen::Matrix<float, 4, 2> &_xys,
    const std::vector<float> &_values);

  /// \brief 1D linear interpolation, given 4 points on a line. Finds 2 points
  /// on a segment within which the query point lies, then interpolates using
  /// them.
  /// \param[in] _p Position to interpolate for
  /// \param[in] _xs Positions to interpolate from
  /// \param[in] _values Data values at the positions to interpolate from
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float BarycentricInterpolate(
    const float &_p,
    const Eigen::VectorXf &_xs,
    const std::vector<float> &_values);

  /// \brief Sort vector, store indices to original vector after sorting.
  /// Original vector unchanged.
  /// \param[in] _v Vector to sort
  /// \param[out] _idx Indices of original vector in sorted vector
  public: template<typename T>
  void SortIndices(
      const std::vector<T> &_v,
      std::vector<size_t> &_idx);

  //////////////////////////////////
  // Hybrid interpolation. 2D triangle barycentric interpolation on 2 z slices,
  // then linear interpolation.

  /// \brief Two 2D barycentric interpolations on two z slices, then a linaer
  /// interpolation between the two intermediate results.
  /// \param[in] _p Position within the tetrahedra to interpolate for
  /// \param[in] _xyzs n x 8. XYZ coordinates of 2 sets of 4 vertices, each
  /// set on a different z slice. [0]-[3] is first slice, [4]-[7] is 2nd slice.
  /// \param[in] _values Data values at the vertices
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float BaryLinearInterpolate(
    const Eigen::Vector3f &_p,
    const Eigen::MatrixXf &_xyzs,
    const std::vector<float> &_values);

  //////////////////////////////////
  /// Utility

  /// \brief Debug printouts for interpolation. Will keep around at least until
  /// interpolation is stable.
  public: const bool DEBUG_INTERPOLATE = true;

  /// \brief Debug printouts for interpolation math. Will keep around at least
  /// until interpolation is stable.
  public: const bool DEBUG_INTERPOLATE_MATH = false;
};

/////////////////////////////////////////////////
Interpolation::Interpolation()
  : dataPtr(std::make_unique<InterpolationPrivate>())
{
}

/////////////////////////////////////////////////
float Interpolation::InterpolateData(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{
  if (this->INTERPOLATION_METHOD == TRILINEAR)
  {
    return this->dataPtr->TrilinearInterpolate(_p, _xyzs, _values);
  }
  else if (this->INTERPOLATION_METHOD == BARYCENTRIC)
  {
    return this->dataPtr->BarycentricInterpolate(_p, _xyzs, _values);
  }
  else
  {
    ignerr << "INTERPOLATION_METHOD value invalid. "
      << "Choose a valid interpolation method." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }
}

/////////////////////////////////////////////////
void Interpolation::FindTrilinearInterpolators(
  pcl::PointCloud<pcl::PointXYZ> &_cloud,
  pcl::PointXYZ &_pt,
  std::vector<int> &_inds,
  std::vector<float> &_sqrDists,
  float _spatialRes,
  std::vector<int> &_interpolatorInds1,
  std::vector<pcl::PointXYZ> &_interpolators1,
  std::vector<int> &_interpolatorInds2,
  std::vector<pcl::PointXYZ> &_interpolators2,
  int _k)
{
  // Initialize return parameters
  _interpolatorInds1.clear();
  _interpolators1.clear();
  _interpolatorInds2.clear();
  _interpolators2.clear();

  // Sanity checks
  // Point cloud not populated
  if (_cloud.size() == 0)
  {
    return;
  }
  if (_inds.size() == 0 || _sqrDists.size() == 0)
  {
    ignwarn << "FindTrilinearInterpolators(): Invalid neighbors array size ("
            << _inds.size() << " and " << _sqrDists.size()
            << "). No neighbors to use for interpolation. Returning NaN."
            << std::endl;
    return;
  }
  if (_inds.size() != _sqrDists.size())
  {
    ignwarn << "FindTrilinearInterpolators(): Number of neighbors != number of "
            << "distances. Invalid input. Returning NaN." << std::endl;
    return;
  }

  // Two slices of different depth z values
  pcl::PointCloud<pcl::PointXYZ> zSlice1, zSlice2;
  // Indices of points in the z slices
  std::vector<int> zSliceInds1, zSliceInds2;

  // Distances of neighboring points in the search results
  // 4 above, 4 below
  std::vector<float> interpolatorSqrDists1, interpolatorSqrDists2;

  // Step 1: 1st NN, in its z slice, search for 4 NNs

  // 1st nearest neighbor
  // The distances from kNN search are sorted, so can always just take [0]th
  int nnIdx = _inds[0];
  float minDist = _sqrDists[0];
  // Get z of neighbor
  float nnZ = _cloud.at(nnIdx).z;

  // Debug output
  if (this->dataPtr->DEBUG_INTERPOLATE)
    igndbg << _cloud.size()
      << " points in full cloud" << std::endl;

  // Create a sub-cloud containing just the z slice
  this->dataPtr->CreateDepthSlice(nnZ, _cloud, zSlice1,
    zSliceInds1);
  if (this->dataPtr->DEBUG_INTERPOLATE)
  {
    igndbg << "1st nn ("
      << _cloud.at(nnIdx).x << ", "
      << _cloud.at(nnIdx).y << ", "
      << _cloud.at(nnIdx).z << "), "
      << "idx " << nnIdx << ", dist " << sqrt(minDist)
      << ", z slice " << zSlice1.points.size() << " points" << std::endl;

    // igndbg << "FindTrilinearInterpolators(): 1st z slice has indices: "
    //   << std::endl;
    // for (int i = 0; i < zSliceInds1.size(); ++i)
    //   igndbg << zSliceInds1[i] << " " << std::endl;
  }

  // Search in z slice for 4 nearest neighbors in this slice
  std::vector<int> interpolatorInds1NewCloud;
  this->dataPtr->CreateAndSearchOctree(_pt, zSlice1, _spatialRes,
    interpolatorInds1NewCloud, interpolatorSqrDists1, _interpolators1, _k);
  if (interpolatorInds1NewCloud.size() < _k ||
    interpolatorSqrDists1.size() < _k)
  {
    ignwarn << "Could not find enough neighbors in 1st slice z = " << nnZ
      << " for trilinear interpolation." << std::endl;
    return;
  }

  // Map back to the indices in the original point cloud for returning
  _interpolatorInds1.clear();
  for (int i = 0; i < interpolatorInds1NewCloud.size(); ++i)
  {
    _interpolatorInds1.push_back(zSliceInds1[interpolatorInds1NewCloud[i]]);
  }

  // Step 2: exclude z slice of 1st NN from further searches.

  // Maps from indices in the new point cloud with 1st z slice removed, to
  // indices in the original point cloud. This is needed for returning
  // second set of interpolator indices to map back to the original cloud.
  std::vector<int> newToOldInds;

  // Create a sub-cloud without points in the z slice of the 1st NN, so that the
  // 2nd NN will be found in another z slice.
  // Set invert flag to get all but the depth slice.
  pcl::PointCloud<pcl::PointXYZ> cloudExceptZSlice1;

  // Convert input indices to PCL type
  pcl::PointIndices::Ptr zSliceInds1pcl(new pcl::PointIndices());
  zSliceInds1pcl->indices = zSliceInds1;

  // Remove all points in the z slice of the 1st NN, so that the 2nd NN will be
  // found in another z slice.
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(_cloud.makeShared());
  extract.setIndices(zSliceInds1pcl);
  extract.setNegative(true);
  extract.filter(cloudExceptZSlice1);
  extract.filter(newToOldInds);

  if (this->dataPtr->DEBUG_INTERPOLATE)
  {
    igndbg << "Excluding 1st nn z slice. Remaining cloud has "
      << cloudExceptZSlice1.points.size() << " points" << std::endl;

    // igndbg << "FindTrilinearInterpolators(): cloud except 1st z slice has "
    //   << "indices:" << std::endl;
    // for (int i = 0; i < newToOldInds.size(); ++i)
    //   igndbg << newToOldInds[i] << " " << std::endl;
  }

  // Step 3: Look for 2nd NN everywhere except z slice of 1st NN.
  // In this 2nd z-slice, search for 4 NNs

  // Search for 2nd NN
  std::vector<int> inds2;
  std::vector<float> sqrDists2;
  std::vector<pcl::PointXYZ> nbrs2;
  this->dataPtr->CreateAndSearchOctree(_pt, cloudExceptZSlice1, _spatialRes,
    inds2, sqrDists2, nbrs2, 1);
  if (inds2.size() < 1 || sqrDists2.size() < 1)
  {
    ignwarn << "Could not find 2nd NN among "
      << cloudExceptZSlice1.points.size()
      << " points for trilinear interpolation" << std::endl;
    return;
  }

  // Take closest point as 2nd NN
  int nnIdx2 = inds2[0];
  float minDist2 = sqrDists2[0];
  // Get z of neighbor
  float nnZ2 = nbrs2[0].z;

  // Step 4: Look for 4 NNs in the z slice of 2nd NN

  // Create a sub-sub-cloud containing just the z slice
  this->dataPtr->CreateDepthSlice(nnZ2, cloudExceptZSlice1, zSlice2,
    zSliceInds2);
  if (this->dataPtr->DEBUG_INTERPOLATE)
  {
    igndbg << "2nd nn ("
      << _cloud.at(nnIdx2).x << ", "
      << _cloud.at(nnIdx2).y << ", "
      << _cloud.at(nnIdx2).z << "), "
      << "idx " << nnIdx2 << ", dist " << sqrt(minDist2)
      << ", z slice " << zSlice2.points.size() << " points" << std::endl;

    // igndbg << "FindTrilinearInterpolators(): 2nd z slice has indices: "
    //   << std::endl;
    // for (int i = 0; i < zSliceInds2.size(); ++i)
    //   igndbg << newToOldInds[zSliceInds2[i]] << " " << std::endl;
  }

  // Search in z slice of 1st NN for 4 nearest neighbors in this slice
  std::vector<int> interpolatorInds2NewCloud;
  this->dataPtr->CreateAndSearchOctree(_pt, zSlice2, _spatialRes,
    interpolatorInds2NewCloud, interpolatorSqrDists2, _interpolators2, _k);
  if (interpolatorInds2NewCloud.size() < _k ||
    interpolatorSqrDists2.size() < _k)
  {
    ignwarn << "Could not find enough neighbors in 2nd slice z = " << nnZ2
      << " for trilinear interpolation." << std::endl;
    return;
  }

  // Map back to the indices in the original point cloud for returning
  _interpolatorInds2.clear();
  for (int i = 0; i < interpolatorInds2NewCloud.size(); ++i)
  {
    _interpolatorInds2.push_back(
      newToOldInds[zSliceInds2[interpolatorInds2NewCloud[i]]]);
    // igndbg << "interpolatorInds2NewCloud[i]: "
    //   << interpolatorInds2NewCloud[i] << std::endl;
    // igndbg << "zSliceInds2[interpolatorInds2NewCloud[i]]: "
    //   << zSliceInds2[interpolatorInds2NewCloud[i]] << std::endl;
    // igndbg << "newToOldInds[zSliceInds2[interpolatorInds2NewCloud[i]]]: "
    //   << newToOldInds[zSliceInds2[interpolatorInds2NewCloud[i]]] << std::endl;
  }

  // if (this->dataPtr->DEBUG_INTERPOLATE)
  // {
  //   igndbg << "FindTrilinearInterpolators(): result indices:" << std::endl;
  //   for (int i = 0; i < _interpolatorInds1.size(); ++i)
  //     igndbg << _interpolatorInds1[i] << " " << std::endl;
  //   for (int i = 0; i < _interpolatorInds2.size(); ++i)
  //     igndbg << _interpolatorInds2[i] << " " << std::endl;
  // }
}

/////////////////////////////////////////////////
bool Interpolation::debug()
{
  return this->dataPtr->DEBUG_INTERPOLATE;
}

/////////////////////////////////////////////////
bool InterpolationPrivate::comparePclPoints(
  const pcl::PointXYZ &a,
  const pcl::PointXYZ &b)
{
  // Comparison between points is arbitrary. This function is only used for
  // set operations, not for literal sorting.
  return a.x < b.x && a.y < b.y && a.z < b.z;
}

/////////////////////////////////////////////////
void InterpolationPrivate::CreateDepthSlice(
  float _depth,
  pcl::PointCloud<pcl::PointXYZ> &_cloud,
  pcl::PointCloud<pcl::PointXYZ> &_zSlice,
  std::vector<int> &_zSliceInds,
  bool _invert)
{
  // Separate a z slice, i.e. points with z equal to that of 1st NN
  // Pass in _invert=true to get indices of removed points
  pcl::PassThrough<pcl::PointXYZ> passThruFilter =
    pcl::PassThrough<pcl::PointXYZ>(true);
  passThruFilter.setInputCloud(_cloud.makeShared());
  passThruFilter.setNegative(_invert);
  passThruFilter.setFilterFieldName("z");
  passThruFilter.setFilterLimits(_depth - 1e-6, _depth + 1e-6);
  passThruFilter.filter(_zSlice);
  passThruFilter.filter(_zSliceInds);
}

/////////////////////////////////////////////////
void InterpolationPrivate::CreateAndSearchOctree(
  pcl::PointXYZ &_searchPt,
  pcl::PointCloud<pcl::PointXYZ> &_cloud,
  float _spatialRes,
  std::vector<int> &_nbrInds,
  std::vector<float> &_nbrSqrDists,
  std::vector<pcl::PointXYZ> &_nbrs,
  int _k)
{
  // Initialize return value populated by hand
  _nbrs.clear();

  // Create octree for cloud
  auto octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(
    _spatialRes);
  octree.setInputCloud(_cloud.makeShared());
  octree.addPointsFromInputCloud();

  // Search in the depth slice to find 4 closest neighbors
  if (octree.getLeafCount() > 0)
  {
    if (octree.nearestKSearch(
      _searchPt, _k, _nbrInds, _nbrSqrDists) <= 0)
    {
      ignwarn << "No data found near search point " << _searchPt
        << std::endl;
      return;
    }
    else
    {
      igndbg << "Found " << _nbrInds.size() << " neighbors."
        << std::endl;

      for (std::size_t i = 0; i < _nbrInds.size(); ++i)
      {
        pcl::PointXYZ nbrPt = _cloud.at(_nbrInds[i]);
        _nbrs.push_back(nbrPt);

        igndbg << "Neighbor at ("
          << std::round(nbrPt.x * 1000.0) / 1000.0 << ", "
          << std::round(nbrPt.y * 1000.0) / 1000.0 << ", "
          << std::round(nbrPt.z * 1000.0) / 1000.0 << "), "
          << "distance " << sqrt(_nbrSqrDists[i]) << " m" << std::endl;
      }
    }
  }
  else
  {
    ignwarn << "Zero points in this octree." << std::endl;
  }
}

/////////////////////////////////////////////////
float InterpolationPrivate::TrilinearInterpolate(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{
  IGN_PROFILE("InterpolationPrivate::TrilinearInterpolate");

  // Sanity check: Must have 8 points, 4 above, 4 below.
  if (_xyzs.rows() != 8)
  {
    ignerr << "Size of interpolators invalid (" << _xyzs.size() << "). "
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
      return this->BaryLinearInterpolate(_p, _xyzs,_values);
    }
  }

  if (this->DEBUG_INTERPOLATE)
    igndbg << "Trilinear interpolation, starting with 8 points: "
      << d000 << ", " << d001 << ", " << d010 << ", " << d011 << ", "
      << d100 << ", " << d101 << ", " << d110 << ", " << d111
      << std::endl;

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

  if (this->DEBUG_INTERPOLATE)
    igndbg << "Trilinear interpolation, 4 intermediate results from 8 points: "
      << d00 << ", " << d01 << ", " << d10 << ", " << d11 << std::endl;

  // Interpolate along y
  // For 4 interpolated points on x above, there are 2 pairs of y's
  float d0 = d00 * (1 - dy) + d10 * dy;
  float d1 = d01 * (1 - dy) + d11 * dy;

  if (this->DEBUG_INTERPOLATE)
    igndbg << "Trilinear interpolation, 2 intermediate results from 4 points: "
      << d0 << ", " << d1 << std::endl;

  // Interpolate along z
  // For 2 interpolated points on y above, there is only 1 z.
  // This arrives at the interpolated value at the target xyz position.
  float d = d0 * (1 - dz) + d1 * dz;

  if (this->DEBUG_INTERPOLATE)
    igndbg << "Trilinear interpolation result: " << d << std::endl;

  return d;
}

/////////////////////////////////////////////////
float InterpolationPrivate::BarycentricInterpolate(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{
  // Implemented from https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Barycentric_coordinates_on_tetrahedra

  if (this->DEBUG_INTERPOLATE_MATH)
    igndbg << "_p: " << std::endl << _p << std::endl;

  Eigen::Matrix3f T;
  // Row 1 is x-coords: x1 - x4, x2 - x4, x3 - x4
  T << _xyzs(0, 0) - _xyzs(3, 0),
       _xyzs(1, 0) - _xyzs(3, 0),
       _xyzs(2, 0) - _xyzs(3, 0),
  // Row 2 is y-coords: y1 - y4, y2 - y4, y3 - y4
       _xyzs(0, 1) - _xyzs(3, 1),
       _xyzs(1, 1) - _xyzs(3, 1),
       _xyzs(2, 1) - _xyzs(3, 1),
  // Row 3 is z-coords: z1 - z4, z2 - z4, z3 - z4
       _xyzs(0, 2) - _xyzs(3, 2),
       _xyzs(1, 2) - _xyzs(3, 2),
       _xyzs(2, 2) - _xyzs(3, 2);
  if (this->DEBUG_INTERPOLATE_MATH)
    igndbg << "T: " << std::endl << T << std::endl;

  int zeroRowCount = 0;
  bool rowIsZero [3] = {false, false, false};
  for (int r = 0; r < T.rows(); ++r)
  {
    if ((T.row(r).array().abs() < 1e-6).all())
    {
      zeroRowCount++;
      rowIsZero[r] = true;
    }
  }

  // If exactly 1 row of T is all zeros, then the points are in a 2D plane.
  // 2D. Interpolate on a plane. Otherwise T inverse will result in nans.
  if (zeroRowCount == 1)
  {
    if (this->DEBUG_INTERPOLATE)
      igndbg << "4 points are on a plane. Using 2D barycentric interpolation "
        "for a triangle." << std::endl;

    // Eliminate the constant axis
    Eigen::Vector2f p2D;
    Eigen::Matrix<float, 4, 2> xyzs2D;
    int nextCol = 0;
    for (int r = 0; r < T.rows(); ++r)
    {
      if (!rowIsZero[r])
      {
        // Populate the axes corresponding to nonzero rows of T.
        // E.g. If row 1 of T is zeros, then points are on x-plane. Ignore
        // x-coordinates, which are on column 1 of the original points matrix.
        p2D(nextCol) = _p(r);
        xyzs2D.col(nextCol) = _xyzs.col(r);
        ++nextCol;
      }
    }
    return this->BarycentricInterpolate(p2D, xyzs2D, _values);
  }
  // 1D. Interpolate on a line. Otherwise T inverse will result in nans.
  else if (zeroRowCount == 2)
  {
    if (this->DEBUG_INTERPOLATE)
      igndbg << "4 points are on a line. Using 1D interpolation." << std::endl;

    float p1D;
    Eigen::VectorXf xyzs1D(_xyzs.rows());
    for (int r = 0; r < T.rows(); ++r)
    {
      // Only one row is non-zero
      if (!rowIsZero[r])
      {
        p1D = _p(r);
        xyzs1D = _xyzs.col(r);
      }
    }
    return this->BarycentricInterpolate(p1D, xyzs1D, _values);
  }
  // T is entirely zero. Then all points are at the same point. Take any value.
  else if (zeroRowCount == 3)
  {
    if (this->DEBUG_INTERPOLATE)
      igndbg << "4 points are at the exact same point. Arbitrarily selecting "
        "one of their values as interpolation result." << std::endl;
    return _values[0];
  }

  // r4 = (x4, y4, z4)
  Eigen::Vector3f r4;
  r4 << _xyzs(3, 0), _xyzs(3, 1), _xyzs(3, 2);
  if (this->DEBUG_INTERPOLATE_MATH)
    igndbg << "r4: " << std::endl << r4 << std::endl;

  // (lambda1, lambda2, lambda3)
  Eigen::Vector3f lambda123 = T.inverse() * (_p - r4);

  if (this->DEBUG_INTERPOLATE_MATH)
    igndbg << "T.inverse(): " << std::endl << T.inverse() << std::endl;

  // lambda4 = 1 - lambda1 - lambda2 - lambda3
  float lambda4 = 1 - lambda123(0) - lambda123(1) - lambda123(2);

  if (this->DEBUG_INTERPOLATE)
    igndbg << "Barycentric 3D lambda 1 2 3 4: " << lambda123(0) << ", "
      << lambda123(1) << ", "
      << lambda123(2) << ", "
      << lambda4 << std::endl;

  // f(r) = lambda1 * f(r1) + lambda2 * f(r2) + lambda3 * f(r3)
  float result =
    lambda123(0) * _values[0] +
    lambda123(1) * _values[1] +
    lambda123(2) * _values[2] +
    lambda4 * _values[3];

  if (this->DEBUG_INTERPOLATE)
    igndbg << "Barycentric 3D interpolation of values " << _values[0] << ", "
      << _values[1] << ", " << _values[2] << ", " << _values[3]
      << " resulted in " << result << std::endl;

  return result;
}

/////////////////////////////////////////////////
float InterpolationPrivate::BarycentricInterpolate(
  const Eigen::Vector2f &_p,
  const Eigen::Matrix<float, 4, 2> &_xys,
  const std::vector<float> &_values)
{
  if (this->DEBUG_INTERPOLATE_MATH)
  {
    igndbg << "_p: " << std::endl << _p << std::endl;
    igndbg << "_xys: " << std::endl << _xys << std::endl;
  }

  // 2D case, consider inputs a triangle and use 2 x 2 matrix for T
  Eigen::Matrix2f T(2, 2);
  Eigen::Vector2f lastVert;
  Eigen::Vector2f lambda12;
  float lambda3;

  // Eliminate the correct point, so that we have a triangle that the query
  // point lies within.
  for (int r = 0; r < _xys.rows(); ++r)
  {
    Eigen::Matrix<float, 3, 2> xys3;
    int nextRow = 0;
    // Populate temp matrix with all points except current point (row)
    for (int r2 = 0; r2 < xys3.rows(); ++r2)
    {
      if (r2 == r)
      {
        continue;
      }
      xys3.row(nextRow++) = _xys.row(r2);
    }
    if (this->DEBUG_INTERPOLATE_MATH)
      igndbg << "xys3: " << std::endl << xys3 << std::endl;

    // Row 1: x1 - x3, x2 - x3
    T << xys3(0, 0) - xys3(2, 0),
         xys3(1, 0) - xys3(2, 0),
    // Row 2: y1 - y3, y2 - y3
         xys3(0, 1) - xys3(2, 1),
         xys3(1, 1) - xys3(2, 1);
    if (this->DEBUG_INTERPOLATE_MATH)
      igndbg << "T: " << std::endl << T << std::endl;

    // lastVert = (x3, y3)
    lastVert << xys3(2, 0), xys3(2, 1);
    if (this->DEBUG_INTERPOLATE_MATH)
      igndbg << "lastVert: " << std::endl << lastVert << std::endl;

    // (lambda1, lambda2)
    lambda12 = T.inverse() * (_p - lastVert);

    if (this->DEBUG_INTERPOLATE_MATH)
      igndbg << "T.inverse(): " << std::endl << T.inverse() << std::endl;

    // lambda3 = 1 - lambda1 - lambda2
    lambda3 = 1 - lambda12(0) - lambda12(1);

    if (this->DEBUG_INTERPOLATE)
      igndbg << "Barycentric 2D lambda 1 2 3: " << lambda12(0) << ", "
        << lambda12(1) << ", "
        << lambda3 << std::endl;

    // If all lambdas >= 0, then we found a triangle that the query point
    // lies within. (A lambda would be negative if point is outside triangle)
    if ((lambda12.array() >= 0).all() && lambda3 >= 0)
    {
      break;
    }
  }

  // f(r) = lambda1 * f(r1) + lambda2 * f(r2)
  float result =
    lambda12(0) * _values[0] +
    lambda12(1) * _values[1] +
    lambda3 * _values[2];

  if (this->DEBUG_INTERPOLATE)
    igndbg << "Barycentric 2D interpolation of values " << _values[0] << ", "
      << _values[1] << ", " << _values[2]
      << " resulted in " << result << std::endl;

  return result;
}

/////////////////////////////////////////////////
float InterpolationPrivate::BarycentricInterpolate(
  const float &_p,
  const Eigen::VectorXf &_xs,
  const std::vector<float> &_values)
{
  if (this->DEBUG_INTERPOLATE_MATH)
  {
    igndbg << "_p: " << std::endl << _p << std::endl;
    igndbg << "_xs: " << std::endl << _xs << std::endl;
  }

  // If _p is equal to one of the points, just take the value of that point.
  // This is to catch floating point errors if _p lies on one side of all
  // points in _xs, but really equal to one of the endpoints.
  if (((_xs.array() - _p).abs() < 1e-6).any())
  {
    for (int i = 0; i < _xs.size(); ++i)
    {
      if (abs(_xs(i) - _p) < 1e-6)
      {
        if (this->DEBUG_INTERPOLATE)
          igndbg << "_p lies on a neighbor. "
            << "1D linear interpolation of values " << _values[0] << ", "
            << _values[1] << ", " << _values[2] << ", " << _values[3]
            << " resulted in " << _values[i] << std::endl;
        return _values[i];
      }
    }
  }

  // If _p lies on one side of all points in _xs, then cannot interpolate.
  if ((_xs.array() - _p < 0).all() ||
      (_xs.array() - _p > 0).all())
  {
    ignwarn << "1D linear interpolation: query point lies on one side of all "
      "interpolation points. Cannot interpolate." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Sort points and store the indices
  std::vector<float> xsSorted;
  std::vector<size_t> xsSortedInds;
  for (int i = 0; i < _xs.size(); ++i)
  {
    xsSorted.push_back(_xs(i));
  }
  SortIndices(xsSorted, xsSortedInds);
  // Access sorted indices to get the new sorted array
  for (int i = 0; i < xsSortedInds.size(); ++i)
  {
    xsSorted[i] = _xs(xsSortedInds[i]);
  }

  int ltPSortedIdx{-1};
  int gtPSortedIdx{-1};
  float ltPDist, gtPDist;

  // Get the two closest positions in _xs that _p lies between.
  for (int i = 0; i < xsSorted.size() - 1; ++i)
  {
    // Two consecutive elements in the sorted vector, that the query point lies
    // between, are the closest points to each side of the query point.
    if (xsSorted[i] <= _p && _p <= xsSorted[i+1])
    {
      ltPSortedIdx = i;
      gtPSortedIdx = i + 1;

      ltPDist = _p - xsSorted[i];
      gtPDist = xsSorted[i+1] - _p;

      break;
    }
  }

  // Sanity check
  if (ltPSortedIdx < 0 || ltPSortedIdx >= xsSortedInds.size() ||
      gtPSortedIdx < 0 || gtPSortedIdx >= xsSortedInds.size())
  {
    ignwarn << "1D linear interpolation: cannot find pair of consecutive "
      << "neighbors that query point lies between. Cannot interpolate. "
      << "(This should not happen!)"
      << std::endl;
    if (this->DEBUG_INTERPOLATE)
    {
      igndbg << "Neighbors: " << std::endl << _xs << std::endl;
      igndbg << "Sorted neighbors: " << std::endl;
      for (int i = 0; i < xsSorted.size(); ++i)
        igndbg << xsSorted[i] << std::endl;
      igndbg << "Query point: " << std::endl << _p << std::endl;
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Normalize the distances to ratios between 0 and 1, to use as weights
  float ltPWeight = ltPDist / (gtPDist + ltPDist);
  float gtPWeight = gtPDist / (gtPDist + ltPDist);

  // Retrieve indices of sorted elements in original array
  int ltPIdx = xsSortedInds[ltPSortedIdx];
  int gtPIdx = xsSortedInds[gtPSortedIdx];

  // Sanity check
  if (ltPIdx >= _values.size() || gtPIdx >= _values.size())
  {
    ignwarn << "1D linear interpolation: mapping from sorted index to "
      << "original index resulted in invalid index. Cannot interpolate. "
      << "(This should not happen!)"
      << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Linear interpolation
  float result = ltPWeight * _values[ltPIdx] + gtPWeight * _values[gtPIdx];

  if (this->DEBUG_INTERPOLATE)
  {
    igndbg << "ltPWeight: " << ltPWeight << ", gtPWeight: " << gtPWeight
      << std::endl;
    igndbg << "1D linear interpolation of values " << _values[0] << ", "
      << _values[1] << ", " << _values[2] << ", " << _values[3]
      << " resulted in " << result << std::endl;
  }

  return result;
}

/////////////////////////////////////////////////
template<typename T>
void InterpolationPrivate::SortIndices(
  const std::vector<T> &_v,
  std::vector<size_t> &_idx)
{
  // From https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes

  // Initialize original index locations
  _idx.resize(_v.size());
  std::iota(_idx.begin(), _idx.end(), 0);

  // Sort indexes based on comparing values in v using std::stable_sort instead
  // of std::sort to avoid unnecessary index re-orderings when v contains
  // elements of equal values
  std::stable_sort(_idx.begin(), _idx.end(),
    [&_v](size_t _i1, size_t _i2) {return _v[_i1] < _v[_i2];});
}

/////////////////////////////////////////////////
float InterpolationPrivate::BaryLinearInterpolate(
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
  float valueSlice1 = this->BarycentricInterpolate(p2d, xysSlice1, _values);
  float valueSlice2 = this->BarycentricInterpolate(p2d, xysSlice2, _values);
  if (this->DEBUG_INTERPOLATE)
    igndbg << "Hybrid bary-linear interpolation, "
      << "2 barycentric interpolations on 2 z slices resulted in: "
      << valueSlice1 << " and " << valueSlice2 << std::endl;

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
  if (this->DEBUG_INTERPOLATE)
    igndbg << "Hybrid bary-linear interpolation, "
      << "linear interpolation of 2 points on two z slices: " << result
      << std::endl;

  if (this->DEBUG_INTERPOLATE)
  {
    igndbg << "Hybrid bary-linear interpolation of " << _values.size()
      << " values:" << std::endl;
    for (int i = 0; i < _values.size(); ++i)
      igndbg << _values[i] << std::endl;
    igndbg << "resulted in " << result << std::endl;
  }

  return result;
}
}
#endif
