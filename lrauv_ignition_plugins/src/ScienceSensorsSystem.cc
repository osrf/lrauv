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

#include <mutex>

#include <ignition/msgs/pointcloud_packed.pb.h>

#include <ignition/common/Profiler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include "ScienceSensorsSystem.hh"

using namespace tethys;

class tethys::ScienceSensorsSystemPrivate
{
  public: enum interpolation
  {
    TRILINEAR,
    BARYCENTRIC
  };

  //////////////////////////////////
  // Functions for data manipulation

  /// \brief Reads csv file and populate various data fields
  /// \param[in] _ecm Immutable reference to the ECM
  public: void ReadData(const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Generate octree from spatial data, for searching
  public: void GenerateOctrees();

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

  /// \brief True to use trilinear interpolation, false to use barycentric
  /// interpolation
  public: int INTERPOLATION_METHOD = TRILINEAR;

  /// \brief Convert a vector of PCL points to an Eigen::Matrix.
  /// \param[in] _vec Source vector
  /// \param[out] _mat Result matrix
  public: void PclVectorToEigen(
    const std::vector<pcl::PointXYZ> &_vec,
    Eigen::MatrixXf &_mat);

  //////////////////////////////////
  // Trilinear interpolation

  /// \brief Comparison function for std::set_difference().
  /// Comparison between points is arbitrary. This function is only used for
  /// set operations, not for literal sorting.
  public: bool comparePclPoints(
    const pcl::PointXYZ &a,
    const pcl::PointXYZ &b);

  /// \brief Find XYZ locations of points in the two closest z slices to
  /// interpolate among.
  /// \param[in] _pt Location in space to interpolate for
  /// \param[in] _inds Indices of nearest neighbors to _pt, used to look for
  /// first z slice.
  /// \param[in] _sqrDists Distances of nearest neighbors to _pt
  /// \param[out] _interpolatorInds2 Indices of points in fist z slice
  /// \param[out] _interpolators1 XYZ points on a z slice to interpolate among
  /// \param[out] _interpolatorInds1 Indices of points in second z slice
  /// \param[out] _interpolators2 XYZ points on a second z slice to interpolate
  /// among
  /// \param[in] _k Number of nearest neighbors. Default to 4, for trilinear
  /// interpolation between two z slices of 4 points per slice.
  public: void FindTrilinearInterpolators(
    pcl::PointXYZ &_pt,
    std::vector<int> &_inds,
    std::vector<float> &_sqrDists,
    std::vector<int> &_interpolatorInds1,
    std::vector<pcl::PointXYZ> &_interpolators1,
    std::vector<int> &_interpolatorInds2,
    std::vector<pcl::PointXYZ> &_interpolators2,
    int _k=4);

  /// \brief Create a z slice from a point cloud. Slice contains points sharing
  /// the same given z value.
  /// \param[in] _min_depth Minimum z value
  /// \param[in] _max_depth Maximum z value
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
  /// \param[in] _searchPt Location in space to interpolate for
  /// \param[in] _cloud Point cloud to search in
  /// \param[out] _nbrInds Result of octree search, indices of points.
  /// \param[out] _nbrSqrDists Result of octree search, distances.
  /// \param[out] _nbrs XYZ of the k nearest neighbors found.
  /// \param[in] _k Number of nearest neighbors. Default to 4, for trilinear
  /// interpolation between two z slices of 4 points per slice.
  public: void CreateAndSearchOctree(
    pcl::PointXYZ &_searchPt,
    pcl::PointCloud<pcl::PointXYZ> &_cloud,
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

  /// \brief Extract elements at indices _inds from _orig vector.
  /// \param[in] _orig A vector of values to extract
  /// \param[in] _inds Indices of elements to extract
  /// \param[out] _new Extracted values
  public: void ExtractElements(
    const std::vector<float> &_orig,
    const std::vector<int> &_inds,
    std::vector<float> &_new);

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

  //////////////////////////////
  // Functions for communication

  /// \brief Publish the latest point cloud
  public: void PublishData();

  /// \brief Service callback for a point cloud with the latest position data.
  /// \param[in] _res Point cloud to return
  /// \return True
  public: bool PointCloudService(ignition::msgs::PointCloudPacked &_res);

  /// \brief Service callback for a float vector with the latest temperature data.
  /// \param[in] _res Float vector to return
  /// \return True
  public: bool TemperatureService(ignition::msgs::Float_V &_res);

  /// \brief Service callback for a float vector with the latest chlorophyll data.
  /// \param[in] _res Float vector to return
  /// \return True
  public: bool ChlorophyllService(ignition::msgs::Float_V &_res);

  /// \brief Service callback for a float vector with the latest salinity data.
  /// \param[in] _res Float vector to return
  /// \return True
  public: bool SalinityService(ignition::msgs::Float_V &_res);

  /// \brief Returns a point cloud message populated with the latest sensor data
  public: ignition::msgs::PointCloudPacked PointCloudMsg();

  ///////////////////////////////
  // Constants for data manipulation

  /// \brief csv field name for timestamp of data
  public: const std::string TIME {"elapsed_time_second"};

  /// \brief csv field name for latitude
  public: const std::string LATITUDE {"latitude_degree"};

  /// \brief csv field name for longitude
  public: const std::string LONGITUDE {"longitude_degree"};

  /// \brief csv field name for depth
  public: const std::string DEPTH {"depth_meter"};

  /// \brief csv field name for temperature
  public: const std::string TEMPERATURE {"sea_water_temperature_degC"};

  /// \brief csv field name for salinity
  public: const std::string SALINITY {"sea_water_salinity_psu"};

  /// \brief csv field name for chlorophyll
  public: const std::string CHLOROPHYLL {
    "mass_concentration_of_chlorophyll_in_sea_water_ugram_per_liter"};

  /// \brief csv field name for ocean current velocity eastward
  public: const std::string EAST_CURRENT {
    "eastward_sea_water_velocity_meter_per_sec"};

  /// \brief csv field name for ocean current velocity northward
  public: const std::string NORTH_CURRENT {
    "northward_sea_water_velocity_meter_per_sec"};

  ////////////////////////////
  // Fields for bookkeeping

  /// \brief Input data file name, relative to a path Ignition can find in its
  /// environment variables.
  public: std::string dataPath {"2003080103_mb_l3_las.csv"};

  /// \brief Coordinates where sensor location was last interpolated.
  /// Helps to determine whether sensor location needs to be updated
  public: ignition::math::Vector3d lastSensorPosENU = {
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max()};

  /// \brief Distance robot needs to move before another data interpolation
  /// (based on sensor location) takes place.
  /// TODO: Compute resolution from data. See where this constant is used.
  public: const float INTERPOLATE_DIST_THRESH = 5.0;

  /// \brief Debug printouts for interpolation. Will keep around at least until
  /// interpolation is stable.
  public: const bool DEBUG_INTERPOLATE = false;

  ///////////////////////////////
  // Variables for coordinate system

  /// \brief Set to true after the spherical coordinates have been initialized.
  /// This may happen at startup if the SDF file has them hardcoded, or at
  /// runtime when the first vehicle is spawned. Assume the coordinates are
  /// only shifted once.
  public: bool sphericalCoordinatesInitialized{false};

  /// \brief Mutex for writing to world origin association to lat/long
  public: std::mutex mtx;

  //////////////////////////////////
  // Variables for data manipulation

  /// \brief Whether using more than one time slices of data
  public: bool multipleTimeSlices {false};

  /// \brief Index of the latest time slice
  public: int timeIdx {0};

  /// \brief Timestamps to index slices of data
  public: std::vector<float> timestamps;

  /// \brief Spatial coordinates of data
  /// Vector size: number of time slices. Indices correspond to those of
  /// this->timestamps.
  /// Point cloud: spatial coordinates to index science data by location
  /// in the ENU world frame.
  public: std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> timeSpaceCoords;

  /// \brief Resolution of spatial coordinates in meters in octree, for data
  /// search.
  /// TODO Compute resolution from data. Take into account that data often have
  /// variable resolution, so maybe take minimum distance between points in the
  /// point cloud.
  public: float spatialRes = 0.1f;

  /// \brief Octree for data search based on spatial location of sensor. One
  /// octree per point cloud in timeSpaceCoords.
  /// Location stored in ENU world coordinates.
  public: std::vector<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr>
    spatialOctrees;

  /// \brief Science data.
  /// Outer vector size: number of time slices. Indices correspond to those of
  /// this->timestamps.
  /// Inner vector: indices correspond to those of timeSpaceCoords.
  public: std::vector<std::vector<float>> temperatureArr;

  /// \brief Science data. Same size as temperatureArr.
  public: std::vector<std::vector<float>> salinityArr;

  /// \brief Science data. Same size as temperatureArr.
  public: std::vector<std::vector<float>> chlorophyllArr;

  /// \brief Science data. Same size as temperatureArr.
  public: std::vector<std::vector<float>> eastCurrentArr;

  /// \brief Science data. Same size as temperatureArr.
  public: std::vector<std::vector<float>> northCurrentArr;

  //////////////////////////////
  // Variables for communication

  /// \brief World object to access world properties.
  public: ignition::gazebo::World world;

  /// \brief Node for communication
  public: ignition::transport::Node node;

  /// \brief Publisher for point clouds representing positions for science data
  public: ignition::transport::Node::Publisher cloudPub;

  /// \brief Name used for both the point cloud topic and service
  public: std::string cloudTopic {"/science_data"};

  /// \brief Publisher for temperature
  public: ignition::transport::Node::Publisher tempPub;

  /// \brief Publisher for chlorophyll
  public: ignition::transport::Node::Publisher chlorPub;

  /// \brief Publisher for salinity
  public: ignition::transport::Node::Publisher salPub;

  /// \brief Temperature message
  public: ignition::msgs::Float_V tempMsg;

  /// \brief Chlorophyll message
  public: ignition::msgs::Float_V chlorMsg;

  /// \brief Salinity message
  public: ignition::msgs::Float_V salMsg;

  /// \brief Publish a few more times for visualization plugin to get them
  public: int repeatPubTimes = 1;
};

/////////////////////////////////////////////////
/// \brief Helper function to create a sensor according to its type
/// \tparam SensorType A sensor type
/// \param[in] _system Pointer to the science sensors system
/// \param[in] _ecm Mutable reference to the ECM
/// \param[in] _entity Sensor entity
/// \param[in] _custom Custom sensor component
/// \param[in] _parent Parent entity component
template<typename SensorType>
void createSensor(ScienceSensorsSystem *_system,
    ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::gazebo::Entity &_entity,
    const ignition::gazebo::components::CustomSensor *_custom,
    const ignition::gazebo::components::ParentEntity *_parent)
{
  auto type = ignition::sensors::customType(_custom->Data());
  if (SensorType::kTypeStr != type)
  {
    return;
  }
  // Get sensor's scoped name without the world
  auto sensorScopedName = ignition::gazebo::removeParentScope(
      ignition::gazebo::scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _custom->Data();
  data.SetName(sensorScopedName);

  // Default to scoped name as topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/" + SensorType::kTypeStr;
    data.SetTopic(topic);
  }

  ignition::sensors::SensorFactory sensorFactory;
  auto sensor = sensorFactory.CreateSensor<SensorType>(data);
  if (nullptr == sensor)
  {
    ignerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // Set sensor parent
  auto parentName = _ecm.Component<ignition::gazebo::components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  // Set topic on Gazebo
  _ecm.CreateComponent(_entity,
      ignition::gazebo::components::SensorTopic(sensor->Topic()));

  // Keep track of this sensor
  _system->entitySensorMap.insert(std::make_pair(_entity,
      std::move(sensor)));

  igndbg << "Created sensor [" << sensorScopedName << "]"
         << std::endl;
}

/////////////////////////////////////////////////
ScienceSensorsSystem::ScienceSensorsSystem()
  : dataPtr(std::make_unique<ScienceSensorsSystemPrivate>())
{
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::ReadData(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ScienceSensorsSystemPrivate::ReadData");

  if (!this->sphericalCoordinatesInitialized)
  {
    ignerr << "Trying to read data before spherical coordinates were "
           << "initialized." << std::endl;
    return;
  }

  // Lock modifications to world origin spherical association until finish
  // reading and transforming data
  std::lock_guard<std::mutex> lock(mtx);

  std::fstream fs;
  fs.open(this->dataPath, std::ios::in);

  std::vector<std::string> fieldnames;
  std::string line, word, temp;

  // Read field names in first line
  std::getline(fs, line);

  std::stringstream ss(line);

  // Tokenize header line into columns
  while (std::getline(ss, word, ','))
  {
    fieldnames.push_back(word);
  }

  // Read file line by line
  while (std::getline(fs, line))
  {
    std::stringstream ss(line);

    int i = 0;

    // Index of the timestamp in this line of data. Init to invalid index
    int lineTimeIdx = -1;

    // Spatial coordinates of this line of data. Init to NaN before populating
    float latitude = std::numeric_limits<float>::quiet_NaN();
    float longitude = std::numeric_limits<float>::quiet_NaN();
    float depth = std::numeric_limits<float>::quiet_NaN();

    // Science data. Init to NaN before knowing whether timestamp is valid, so
    // that we do not assume timestamp column precedes data columns in each line
    // in the file.
    float temp = std::numeric_limits<float>::quiet_NaN();
    float sal = std::numeric_limits<float>::quiet_NaN();
    float chlor = std::numeric_limits<float>::quiet_NaN();
    float nCurr = std::numeric_limits<float>::quiet_NaN();
    float eCurr = std::numeric_limits<float>::quiet_NaN();

    // Tokenize the line into columns
    while (std::getline(ss, word, ','))
    {
      float val = 0.0f;
      try
      {
        // stof handles NaNs and Infs
        val = stof(word);
      }
      catch (const std::invalid_argument &ia)
      {
        ignerr << "Line [" << line << "] contains invalid word. Skipping. "
               << ia.what() << std::endl;
        continue;
      }
      catch (const std::out_of_range &oor)
      {
        ignerr << "Line [" << line << "] contains invalid word. Skipping. "
               << oor.what() << std::endl;
        continue;
      }

      // Time index
      if (fieldnames[i] == TIME)
      {
        // Does not account for floating point error. Assumes time specified in
        // csv file is same accuracy for each line.
        std::vector<float>::iterator it =
          std::find(this->timestamps.begin(), this->timestamps.end(), val);
        // If the timestamp is new
        if (it == this->timestamps.end())
        {
          // Insert new timestamp into 1D array
          this->timestamps.push_back(val);

          // Create a new time slice of data
          auto newCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>);
          this->timeSpaceCoords.push_back(newCloud->makeShared());
          // Is this valid memory management?
          this->temperatureArr.push_back(std::vector<float>());
          this->salinityArr.push_back(std::vector<float>());
          this->chlorophyllArr.push_back(std::vector<float>());
          this->eastCurrentArr.push_back(std::vector<float>());
          this->northCurrentArr.push_back(std::vector<float>());

          lineTimeIdx = this->timestamps.size() - 1;
        }
        // If the timestamp exists, find the index of its corresponding time
        // slice
        else
        {
          lineTimeIdx = it - this->timestamps.begin();
        }
      }
      // Spatial index: latitude
      else if (fieldnames[i] == LATITUDE)
      {
        latitude = val;
      }
      // Spatial index: longitude
      else if (fieldnames[i] == LONGITUDE)
      {
        longitude = val;
      }
      // Spatial index: depth
      else if (fieldnames[i] == DEPTH)
      {
        depth = val;
      }
      // Science data
      else if (fieldnames[i] == TEMPERATURE)
      {
        temp = val;
      }
      else if (fieldnames[i] == SALINITY)
      {
        sal = val;
      }
      else if (fieldnames[i] == CHLOROPHYLL)
      {
        chlor = val;
      }
      else if (fieldnames[i] == EAST_CURRENT)
      {
        eCurr = val;
      }
      else if (fieldnames[i] == NORTH_CURRENT)
      {
        nCurr = val;
      }
      else
      {
        ignerr << "Unrecognized science data field name [" << fieldnames[i]
               << "]. Skipping column." << std::endl;
      }

      i += 1;
    }

    // Check validity of timestamp
    // If no timestamp was provided for this line, cannot index the datum.
    if (lineTimeIdx == -1)
    {
      ignerr << "Line [" << line << "] timestamp invalid. Skipping."
             << std::endl;
      continue;
    }
    else
    {
      // Check validity of spatial coordinates
      if (!std::isnan(latitude) && !std::isnan(longitude) && !std::isnan(depth))
      {
        // Convert lat / lon / elevation to Cartesian ENU
        auto cart = this->world.SphericalCoordinates(_ecm).value()
            .PositionTransform({IGN_DTOR(latitude), IGN_DTOR(longitude), 0.0},
            ignition::math::SphericalCoordinates::SPHERICAL,
            ignition::math::SphericalCoordinates::LOCAL2);
        // Flip sign of z, because positive depth is negative z.
        cart.Z() = -depth;

        // Gather spatial coordinates, 3 fields in the line, into point cloud
        // for indexing this time slice of data.
        this->timeSpaceCoords[lineTimeIdx]->push_back(
          pcl::PointXYZ(cart.X(), cart.Y(), cart.Z()));

        // Populate science data
        this->temperatureArr[lineTimeIdx].push_back(temp);
        this->salinityArr[lineTimeIdx].push_back(sal);
        this->chlorophyllArr[lineTimeIdx].push_back(chlor);
        this->eastCurrentArr[lineTimeIdx].push_back(eCurr);
        this->northCurrentArr[lineTimeIdx].push_back(nCurr);
      }
      // If spatial coordinates invalid, cannot use to index this datum
      else
      {
        ignerr << "Line [" << line << "] has invalid spatial coordinates "
               << "(latitude, longitude, and/or depth). Skipping." << std::endl;
        continue;
      }
    }
  }

  // Make sure the number of timestamps in the 1D indexing array, and the
  // number of time slices of data, are the same.
  assert(this->timestamps.size() == this->timeSpaceCoords.size());

  for (int i = 0; i < this->timeSpaceCoords.size(); i++)
  {
    igndbg << "At time slice " << this->timestamps[i] << ", populated "
           << this->timeSpaceCoords[i]->size()
           << " spatial coordinates." << std::endl;
  }
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::GenerateOctrees()
{
  // For each time slice, create an octree for the spatial coordinates
  for (int i = 0; i < this->timeSpaceCoords.size(); ++i)
  {
    this->spatialOctrees.push_back(
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr(
        new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(
          this->spatialRes)));

    // Populate octree with spatial coordinates
    this->spatialOctrees[i]->setInputCloud(this->timeSpaceCoords[i]);
    this->spatialOctrees[i]->addPointsFromInputCloud();
  }
}

/////////////////////////////////////////////////
float ScienceSensorsSystemPrivate::InterpolateData(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{
  if (this->INTERPOLATION_METHOD == TRILINEAR)
  {
    return this->TrilinearInterpolate(_p, _xyzs, _values);
  }
  else if (this->INTERPOLATION_METHOD == BARYCENTRIC)
  {
    return this->BarycentricInterpolate(_p, _xyzs, _values);
  }
  else
  {
    ignerr << "INTERPOLATION_METHOD value invalid. "
      << "Choose a valid interpolation method." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::PclVectorToEigen(
  const std::vector<pcl::PointXYZ> &_vec,
  Eigen::MatrixXf &_mat)
{
  _mat = Eigen::MatrixXf(_vec.size(), 3);

  // Convert to Eigen::Matrix. One point per row
  for (int r = 0; r < _vec.size(); ++r)
  {
    _mat.row(r) << _vec.at(r).x, _vec.at(r).y, _vec.at(r).z;
  }
}

/////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::comparePclPoints(
  const pcl::PointXYZ &a,
  const pcl::PointXYZ &b)
{
  // Comparison between points is arbitrary. This function is only used for
  // set operations, not for literal sorting.
  return a.x < b.x && a.y < b.y && a.z < b.z;
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::FindTrilinearInterpolators(
  pcl::PointXYZ &_pt,
  std::vector<int> &_inds,
  std::vector<float> &_sqrDists,
  std::vector<int> &_interpolatorInds1,
  std::vector<pcl::PointXYZ> &_interpolators1,
  std::vector<int> &_interpolatorInds2,
  std::vector<pcl::PointXYZ> &_interpolators2,
  int _k)
{
  // TODO(tfoote) magic numbers
  // Should be passed in and paramaterized based on the expected data 
  // distribution height or calculated from the grandularity of the dataset.
  float slice_depth = 5; // 5 meter. 
  float epsilon = 1e-6; // Enough for not equal

  // Sanity checks
  // Vector not populated
  if (this->timeSpaceCoords.size() == 0)
  {
    return;
  }
  // Point cloud not populated
  if (this->timeSpaceCoords[this->timeIdx]->size() == 0)
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
  float nnZ = this->timeSpaceCoords[this->timeIdx]->at(nnIdx).z;

  // Debug output
  if (this->DEBUG_INTERPOLATE)
    igndbg << this->timeSpaceCoords[this->timeIdx]->size()
      << " points in full cloud" << std::endl;

  // Create a sub-cloud containing just the z slice
  this->CreateDepthSlice(_pt.z, _pt.z + slice_depth, *(this->timeSpaceCoords[this->timeIdx]), zSlice1,
    zSliceInds1);
  if (this->DEBUG_INTERPOLATE)
  {
    igndbg << "1st nn ("
      << this->timeSpaceCoords[this->timeIdx]->at(nnIdx).x << ", "
      << this->timeSpaceCoords[this->timeIdx]->at(nnIdx).y << ", "
      << this->timeSpaceCoords[this->timeIdx]->at(nnIdx).z << "), "
      << "idx " << nnIdx << ", dist " << sqrt(minDist)
      << ", z slice " << zSlice1.points.size() << " points" << std::endl;

    //igndbg << "FindTrilinearInterpolators(): 1st z slice has indices: "
    //  << std::endl;
    //for (int i = 0; i < zSliceInds1.size(); ++i)
    //  igndbg << zSliceInds1[i] << " " << std::endl;
  }

  // Search in z slice for 4 nearest neighbors in this slice
  std::vector<int> interpolatorInds1NewCloud;
  this->CreateAndSearchOctree(_pt, zSlice1,
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
  this->CreateDepthSlice(_pt.z, _pt.z + slice_depth, *(this->timeSpaceCoords[this->timeIdx]),
    cloudExceptZSlice1, newToOldInds, true);
  if (this->DEBUG_INTERPOLATE)
  {
    igndbg << "Excluding 1st nn z slice. Remaining cloud has "
      << cloudExceptZSlice1.points.size() << " points" << std::endl;

    //igndbg << "FindTrilinearInterpolators(): cloud except 1st z slice has "
    //  << "indices:" << std::endl;
    //for (int i = 0; i < newToOldInds.size(); ++i)
    //  igndbg << newToOldInds[i] << " " << std::endl;
  }

  // Step 3: Look for 2nd NN everywhere except z slice of 1st NN.
  // In this 2nd z-slice, search for 4 NNs

  // Search for 2nd NN
  std::vector<int> inds2;
  std::vector<float> sqrDists2;
  std::vector<pcl::PointXYZ> nbrs2;
  this->CreateAndSearchOctree(_pt, cloudExceptZSlice1,
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
  this->CreateDepthSlice(_pt.z - slice_depth, _pt.z - epsilon, cloudExceptZSlice1, zSlice2, zSliceInds2);
  if (this->DEBUG_INTERPOLATE)
  {
    igndbg << "2nd nn ("
      << this->timeSpaceCoords[this->timeIdx]->at(nnIdx2).x << ", "
      << this->timeSpaceCoords[this->timeIdx]->at(nnIdx2).y << ", "
      << this->timeSpaceCoords[this->timeIdx]->at(nnIdx2).z << "), "
      << "idx " << nnIdx2 << ", dist " << sqrt(minDist2)
      << ", z slice " << zSlice2.points.size() << " points" << std::endl;

    //igndbg << "FindTrilinearInterpolators(): 2nd z slice has indices: "
    //  << std::endl;
    //for (int i = 0; i < zSliceInds2.size(); ++i)
    //  igndbg << newToOldInds[zSliceInds2[i]] << " " << std::endl;
  }

  // Search in z slice of 1st NN for 4 nearest neighbors in this slice
  std::vector<int> interpolatorInds2NewCloud;
  this->CreateAndSearchOctree(_pt, zSlice2,
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
    //igndbg << "interpolatorInds2NewCloud[i]: "
    //  << interpolatorInds2NewCloud[i] << std::endl;
    //igndbg << "zSliceInds2[interpolatorInds2NewCloud[i]]: "
    //  << zSliceInds2[interpolatorInds2NewCloud[i]] << std::endl;
    //igndbg << "newToOldInds[zSliceInds2[interpolatorInds2NewCloud[i]]]: "
    //  << newToOldInds[zSliceInds2[interpolatorInds2NewCloud[i]]] << std::endl;
  }

  //if (this->DEBUG_INTERPOLATE)
  //{
  //  igndbg << "FindTrilinearInterpolators(): result indices:" << std::endl;
  //  for (int i = 0; i < _interpolatorInds1.size(); ++i)
  //    igndbg << _interpolatorInds1[i] << " " << std::endl;
  //  for (int i = 0; i < _interpolatorInds2.size(); ++i)
  //    igndbg << _interpolatorInds2[i] << " " << std::endl;
  //}
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::CreateDepthSlice(
  float _min_depth,
  float _max_depth,
  pcl::PointCloud<pcl::PointXYZ> &_cloud,
  pcl::PointCloud<pcl::PointXYZ> &_zSlice,
  std::vector<int> &_zSliceInds,
  bool _invert)
{
  // Separate a z slice, i.e. points with z equal to that of 1st NN
  // Pass in extract_removed_indices=true to get indices of removed points
  pcl::PassThrough<pcl::PointXYZ> passThruFilter =
    pcl::PassThrough<pcl::PointXYZ>(true);
  passThruFilter.setInputCloud(_cloud.makeShared());
  passThruFilter.setNegative(_invert);
  passThruFilter.setFilterFieldName("z");
  passThruFilter.setFilterLimits(_min_depth, _max_depth);
  passThruFilter.filter(_zSlice);
  passThruFilter.filter(_zSliceInds);
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::CreateAndSearchOctree(
  pcl::PointXYZ &_searchPt,
  pcl::PointCloud<pcl::PointXYZ> &_cloud,
  std::vector<int> &_nbrInds,
  std::vector<float> &_nbrSqrDists,
  std::vector<pcl::PointXYZ> &_nbrs,
  int _k)
{
  // Initialize return value populated by hand
  _nbrs.clear();

  // Create octree for cloud
  auto octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(
    this->spatialRes);
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
float ScienceSensorsSystemPrivate::TrilinearInterpolate(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{
  IGN_PROFILE("ScienceSensorsSystemPrivate::TrilinearInterpolate");

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
float ScienceSensorsSystemPrivate::BarycentricInterpolate(
  const Eigen::Vector3f &_p,
  const Eigen::MatrixXf &_xyzs,
  const std::vector<float> &_values)
{
  // Implemented from https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Barycentric_coordinates_on_tetrahedra

  if (this->DEBUG_INTERPOLATE)
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
  if (this->DEBUG_INTERPOLATE)
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
  if (this->DEBUG_INTERPOLATE)
    igndbg << "r4: " << std::endl << r4 << std::endl;

  // (lambda1, lambda2, lambda3)
  Eigen::Vector3f lambda123 = T.inverse() * (_p - r4);

  if (this->DEBUG_INTERPOLATE)
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
float ScienceSensorsSystemPrivate::BarycentricInterpolate(
  const Eigen::Vector2f &_p,
  const Eigen::Matrix<float, 4, 2> &_xys,
  const std::vector<float> &_values)
{
  if (this->DEBUG_INTERPOLATE)
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
    if (this->DEBUG_INTERPOLATE)
      igndbg << "xys3: " << std::endl << xys3 << std::endl;

    // Row 1: x1 - x3, x2 - x3
    T << xys3(0, 0) - xys3(2, 0),
         xys3(1, 0) - xys3(2, 0),
    // Row 2: y1 - y3, y2 - y3
         xys3(0, 1) - xys3(2, 1),
         xys3(1, 1) - xys3(2, 1);
    if (this->DEBUG_INTERPOLATE)
      igndbg << "T: " << std::endl << T << std::endl;

    // lastVert = (x3, y3)
    lastVert << xys3(2, 0), xys3(2, 1);
    if (this->DEBUG_INTERPOLATE)
      igndbg << "lastVert: " << std::endl << lastVert << std::endl;

    // (lambda1, lambda2)
    lambda12 = T.inverse() * (_p - lastVert);

    if (this->DEBUG_INTERPOLATE)
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
float ScienceSensorsSystemPrivate::BarycentricInterpolate(
  const float &_p,
  const Eigen::VectorXf &_xs,
  const std::vector<float> &_values)
{
  if (this->DEBUG_INTERPOLATE)
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
void ScienceSensorsSystemPrivate::ExtractElements(
  const std::vector<float> &_orig,
  const std::vector<int> &_inds,
  std::vector<float> &_new)
{
  _new.clear();

  for (int i = 0; i < _inds.size(); ++i)
  {
    _new.push_back(_orig[_inds[i]]);
  }
}

/////////////////////////////////////////////////
template<typename T>
void ScienceSensorsSystemPrivate::SortIndices(
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
float ScienceSensorsSystemPrivate::BaryLinearInterpolate(
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

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::PublishData()
{
  IGN_PROFILE("ScienceSensorsSystemPrivate::PublishData");
  this->tempPub.Publish(this->tempMsg);
  this->chlorPub.Publish(this->chlorMsg);
  this->salPub.Publish(this->salMsg);

  // Publish cloud last. The floatVs are optional, so if the GUI gets the cloud
  // first it will display a monochrome cloud until it receives the floats
  this->cloudPub.Publish(this->PointCloudMsg());
}

/////////////////////////////////////////////////
void ScienceSensorsSystem::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
  this->dataPtr->world = ignition::gazebo::World(_entity);

  if (_sdf->HasElement("data_path"))
  {
    this->dataPtr->dataPath = _sdf->Get<std::string>("data_path");
  }

  ignition::common::SystemPaths sysPaths;
  std::string fullPath = sysPaths.FindFile(this->dataPtr->dataPath);
  if (fullPath.empty())
  {
     ignerr << "Data file [" << this->dataPtr->dataPath << "] not found."
            << std::endl;
     return;
  }
  else
  {
    this->dataPtr->dataPath = fullPath;
    ignmsg << "Loading science data from [" << this->dataPtr->dataPath << "]"
           << std::endl;
  }

  // Advertise cloud as a service for requests on-demand, and a topic for updates
  this->dataPtr->cloudPub = this->dataPtr->node.Advertise<
      ignition::msgs::PointCloudPacked>(this->dataPtr->cloudTopic);

  this->dataPtr->node.Advertise(this->dataPtr->cloudTopic,
      &ScienceSensorsSystemPrivate::PointCloudService, this->dataPtr.get());

  // Advertise science data, also as service and topics
  std::string temperatureTopic{"/temperature"};
  this->dataPtr->tempPub = this->dataPtr->node.Advertise<
      ignition::msgs::Float_V>(temperatureTopic);
  this->dataPtr->node.Advertise(temperatureTopic,
      &ScienceSensorsSystemPrivate::TemperatureService, this->dataPtr.get());

  std::string chlorophyllTopic{"/chloropyll"};
  this->dataPtr->chlorPub = this->dataPtr->node.Advertise<
      ignition::msgs::Float_V>(chlorophyllTopic);
  this->dataPtr->node.Advertise(chlorophyllTopic,
      &ScienceSensorsSystemPrivate::ChlorophyllService, this->dataPtr.get());

  std::string salinityTopic{"/salinity"};
  this->dataPtr->salPub = this->dataPtr->node.Advertise<
      ignition::msgs::Float_V>(salinityTopic);
  this->dataPtr->node.Advertise(salinityTopic,
      &ScienceSensorsSystemPrivate::SalinityService, this->dataPtr.get());
}

/////////////////////////////////////////////////
void ScienceSensorsSystem::PreUpdate(
  const ignition::gazebo::UpdateInfo &,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  _ecm.EachNew<ignition::gazebo::components::CustomSensor,
               ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *_custom,
        const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        createSensor<SalinitySensor>(this, _ecm, _entity, _custom, _parent);
        createSensor<TemperatureSensor>(this, _ecm, _entity, _custom, _parent);
        createSensor<ChlorophyllSensor>(this, _ecm, _entity, _custom, _parent);
        createSensor<CurrentSensor>(this, _ecm, _entity, _custom, _parent);
        return true;
      });
}

/////////////////////////////////////////////////
void ScienceSensorsSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE_THREAD_NAME("ScienceSensorsSystem PostUpdate");
  IGN_PROFILE("ScienceSensorsSystem::PostUpdate");

  this->RemoveSensorEntities(_ecm);

  if (_info.paused)
    return;

  // Delay reading data and generating octrees until spherical coordinates are
  // received.
  if (!this->dataPtr->sphericalCoordinatesInitialized)
  {
    if (this->dataPtr->world.SphericalCoordinates(_ecm))
    {
      this->dataPtr->sphericalCoordinatesInitialized = true;

      this->dataPtr->ReadData(_ecm);
      this->dataPtr->GenerateOctrees();
    }
    else
    {
      // TODO(chapulina) Throttle if it becomes spammy
      ignwarn << "Science sensor data won't be published because spherical "
              << "coordinates are unknown." << std::endl;
      return;
    }
  }

  double simTimeSeconds = std::chrono::duration_cast<std::chrono::seconds>(
    _info.simTime).count();

  // Update time index
  if (this->dataPtr->multipleTimeSlices)
  {
    // Only update if sim time exceeds the elapsed timestamp in data
    if (!this->dataPtr->timestamps.empty() &&
      simTimeSeconds >= this->dataPtr->timestamps[this->dataPtr->timeIdx])
    {
      // Increment for next point in time
      this->dataPtr->timeIdx++;

      // Publish science data at the next timestamp
      this->dataPtr->PublishData();
    }
  }

  // Publish every n iters so that VisualizePointCloud plugin gets it.
  // Otherwise the initial publication in Configure() is not enough.
  if (this->dataPtr->repeatPubTimes % 10000 == 0)
  {
    this->dataPtr->PublishData();
    // Reset
    this->dataPtr->repeatPubTimes = 1;
  }
  else
  {
    this->dataPtr->repeatPubTimes++;
  }

  // Whether interpolation is needed
  bool reinterpolate = false;

  // Sensor position to interpolate for
  ignition::math::Vector3d sensorPosENU;

  // Barycentric interpolation searches 4 neighbors directly
  int initK = 4;
  // Trilinear interpolation starts by searching for 1 neighbor. Only its z is
  // used, to find the nearest z slice.
  if (this->dataPtr->INTERPOLATION_METHOD == this->dataPtr->TRILINEAR)
  {
    initK = 1;
  }

  // Number of points per z slice, for trilinear interpolation
  int nbrsPerZSlice = 4;

  // Indices and distances of neighboring points to sensor position
  std::vector<int> spatialInds;
  std::vector<float> spatialSqrDists;

  // Positions of neighbors to use in interpolation, and their indices in
  // original point cloud
  std::vector<pcl::PointXYZ> interpolatorXYZs;
  std::vector<int> interpolatorInds;

  // Get a sensor's position, search in the octree for the closest neighbors.
  // Only need to done for one sensor. All sensors are on the robot, doesn't
  // make a big difference to data location.
  for (auto &[entity, sensor] : this->entitySensorMap)
  {
    // Sensor pose in ENU, used to search for data by spatial coordinates
    sensorPosENU = ignition::gazebo::worldPose(entity, _ecm).Pos();
    pcl::PointXYZ searchPoint(
        sensorPosENU.X(),
        sensorPosENU.Y(),
        sensorPosENU.Z());

    // Visualization never shows up when interpolation is always called.
    // Quick fix: Don't need to interpolate EVERY PostUpdate(). That's overkill.
    // Only need to do it after robot has moved a distance from when we did
    // the previous interpolation
    // TODO Replace with something relating to the closest data point.
    // Potentially lastSensorPosENU as well as lastSensorPosDataProximity.
    // Or use spatialRes computed on the data.
    if (sensorPosENU.Distance(this->dataPtr->lastSensorPosENU) <
      this->dataPtr->INTERPOLATE_DIST_THRESH)
    {
      break;
    }

    if (this->dataPtr->DEBUG_INTERPOLATE)
    {
      igndbg << "Searching around sensor Cartesian location "
        << std::round(searchPoint.x * 1000.0) / 1000.0 << ", "
        << std::round(searchPoint.y * 1000.0) / 1000.0 << ", "
        << std::round(searchPoint.z * 1000.0) / 1000.0 << std::endl;
    }

    // TODO: After mabelzhang/interpolate_sci_raw_mat branch is merged, use
    // the opposite condition and break out of loop.
    // If there are any nodes in the octree, search in octree to find spatial
    // index of science data
    if (this->dataPtr->spatialOctrees[this->dataPtr->timeIdx]->getLeafCount()
      > 0)
    {
      IGN_PROFILE("ScienceSensorsSystem::PostUpdate nearestKSearch");

      // kNN search (alternatives are voxel search and radius search. kNN
      // search is good for variable resolution when the distance to the next
      // neighbor is unknown).
      // Search for nearest neighbors
      if (this->dataPtr->spatialOctrees[this->dataPtr->timeIdx]
        ->nearestKSearch(searchPoint, initK, spatialInds, spatialSqrDists) <= 0)
      {
        ignwarn << "Not enough data found near sensor location " << sensorPosENU
          << std::endl;
        continue;
      }
      // Debug output
      else if (this->dataPtr->DEBUG_INTERPOLATE)
      {
        for (std::size_t i = 0; i < spatialInds.size(); i++)
        {
          // Index the point cloud at the current time slice
          pcl::PointXYZ nbrPt = this->dataPtr->timeSpaceCoords[
            this->dataPtr->timeIdx]->at(spatialInds[i]);

          igndbg << "Neighbor at ("
            << std::round(nbrPt.x * 1000) / 1000.0 << ", "
            << std::round(nbrPt.y * 1000) / 1000.0 << ", "
            << std::round(nbrPt.z * 1000) / 1000.0
            << "), squared distance " << spatialSqrDists[i]
            << " m" << std::endl;
        }
      }
      reinterpolate = true;

      if (this->dataPtr->INTERPOLATION_METHOD == this->dataPtr->TRILINEAR)
      {
        // Find 2 sets of 4 nearest neighbors, each set on a different z slice,
        // to use as inputs for trilinear interpolation
        std::vector<pcl::PointXYZ> interpolatorsSlice1, interpolatorsSlice2;
        std::vector<int> interpolatorInds1, interpolatorInds2;
        this->dataPtr->FindTrilinearInterpolators(searchPoint, spatialInds,
          spatialSqrDists, interpolatorInds1, interpolatorsSlice1,
          interpolatorInds2, interpolatorsSlice2, nbrsPerZSlice);

        if (interpolatorsSlice1.size() < nbrsPerZSlice ||
          interpolatorsSlice2.size() < nbrsPerZSlice)
        {
          ignwarn << "Could not find trilinear interpolators near sensor "
            << "location " << sensorPosENU << std::endl;
          continue;
        }

        // Concatenate the 2 sets of 4 points into a vector of 8 points
        interpolatorXYZs.reserve(interpolatorsSlice1.size() +
          interpolatorsSlice2.size());
        interpolatorXYZs.insert(interpolatorXYZs.end(),
          interpolatorsSlice1.begin(), interpolatorsSlice1.end());
        interpolatorXYZs.insert(interpolatorXYZs.end(),
          interpolatorsSlice2.begin(), interpolatorsSlice2.end());

        // Prepare neighbor data to pass to interpolation
        for (int i = 0; i < interpolatorInds1.size(); ++i)
        {
          interpolatorInds.push_back(interpolatorInds1[i]);
        }
        for (int i = 0; i < interpolatorInds2.size(); ++i)
        {
          interpolatorInds.push_back(interpolatorInds2[i]);
        }
      }
      else if (this->dataPtr->INTERPOLATION_METHOD ==
        this->dataPtr->BARYCENTRIC)
      {
        // Prepare neighbor data to pass to interpolation
        for (int i = 0; i < spatialInds.size(); ++i)
        {
          interpolatorInds.push_back(spatialInds[i]);
          interpolatorXYZs.push_back(this->dataPtr->timeSpaceCoords[
            this->dataPtr->timeIdx]->at(spatialInds[i]));
        }
      }
      else
      {
        ignerr << "INTERPOLATION_METHOD value invalid. "
          << "Choose a valid interpolation method." << std::endl;
        break;
      }

      // Update last update position to the current position
      this->dataPtr->lastSensorPosENU = sensorPosENU;
    }

    // Only need to find position ONCE for the entire robot. Don't need to
    // repeat for every sensor.
    break;
  }

  // Convert to Eigen to pass to interpolation
  Eigen::Vector3f sensorPosENUEigen;
  sensorPosENUEigen << sensorPosENU.X(), sensorPosENU.Y(), sensorPosENU.Z();

  Eigen::MatrixXf interpolatorXYZsMat;
  this->dataPtr->PclVectorToEigen(interpolatorXYZs, interpolatorXYZsMat);

  // For each sensor, interpolate using existing data at neighboring positions,
  // to generate data for that sensor.
  for (auto &[entity, sensor] : this->entitySensorMap)
  {
    if (reinterpolate)
    {
      // Input values to interpolation
      std::vector<float> interpolationValues;

      // For the correct sensor, interpolate using nearby locations with data
      if (auto casted = std::dynamic_pointer_cast<SalinitySensor>(sensor))
      {
        if (this->dataPtr->DEBUG_INTERPOLATE)
          igndbg << "Interpolating salinity" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->salinityArr[this->dataPtr->timeIdx],
          interpolatorInds, interpolationValues);
        float sal = this->dataPtr->InterpolateData(
          sensorPosENUEigen, interpolatorXYZsMat, interpolationValues);
        casted->SetData(sal);
      }
      else if (auto casted = std::dynamic_pointer_cast<TemperatureSensor>(
        sensor))
      {
        if (this->dataPtr->DEBUG_INTERPOLATE)
          igndbg << "Interpolating temperature" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->temperatureArr[this->dataPtr->timeIdx],
          interpolatorInds, interpolationValues);
        float temp = this->dataPtr->InterpolateData(
          sensorPosENUEigen, interpolatorXYZsMat, interpolationValues);

        ignition::math::Temperature tempC;
        tempC.SetCelsius(temp);
        casted->SetData(tempC);
      }
      else if (auto casted = std::dynamic_pointer_cast<ChlorophyllSensor>(
        sensor))
      {
        if (this->dataPtr->DEBUG_INTERPOLATE)
          igndbg << "Interpolating chlorophyll" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->chlorophyllArr[this->dataPtr->timeIdx],
          interpolatorInds, interpolationValues);
        float chlor = this->dataPtr->InterpolateData(
          sensorPosENUEigen, interpolatorXYZsMat, interpolationValues);
        casted->SetData(chlor);
      }
      else if (auto casted = std::dynamic_pointer_cast<CurrentSensor>(
        sensor))
      {
        if (this->dataPtr->DEBUG_INTERPOLATE)
          igndbg << "Interpolating E and N currents" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->eastCurrentArr[this->dataPtr->timeIdx],
          interpolatorInds, interpolationValues);
        float eCurr = this->dataPtr->InterpolateData(
          sensorPosENUEigen, interpolatorXYZsMat, interpolationValues);

        // Reset before reuse
        interpolationValues.clear();

        this->dataPtr->ExtractElements(
          this->dataPtr->northCurrentArr[this->dataPtr->timeIdx],
          interpolatorInds, interpolationValues);
        float nCurr = this->dataPtr->InterpolateData(
          sensorPosENUEigen, interpolatorXYZsMat, interpolationValues);

        auto curr = ignition::math::Vector3d(eCurr, nCurr, 0.0);
        casted->SetData(curr);
      }
      else
      {
        ignerr << "Unsupported sensor type, failed to set data" << std::endl;
      }
    }

    // Update all the sensors
    sensor->Update(_info.simTime, false);
  }
}

//////////////////////////////////////////////////
void ScienceSensorsSystem::RemoveSensorEntities(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ScienceSensorsSystem::RemoveSensorEntities");

  _ecm.EachRemoved<ignition::gazebo::components::CustomSensor>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        igndbg << "Removed sensor entity [" << _entity << "]" << std::endl;

        return true;
      });
}

//////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::PointCloudService(
    ignition::msgs::PointCloudPacked &_res)
{
  _res = this->PointCloudMsg();
  return true;
}

//////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::TemperatureService(
    ignition::msgs::Float_V &_res)
{
  _res = this->tempMsg;
  return true;
}

//////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::ChlorophyllService(
    ignition::msgs::Float_V &_res)
{
  _res = this->chlorMsg;
  return true;
}

//////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::SalinityService(
    ignition::msgs::Float_V &_res)
{
  _res = this->salMsg;
  return true;
}

//////////////////////////////////////////////////
ignition::msgs::PointCloudPacked ScienceSensorsSystemPrivate::PointCloudMsg()
{
  IGN_PROFILE("ScienceSensorsSystemPrivate::PointCloudMsg");

  ignition::msgs::PointCloudPacked msg;

  if (this->timeIdx < 0 || this->timeIdx >= this->timestamps.size())
  {
    ignerr << "Invalid time index [" << this->timeIdx << "]."
           << std::endl;
    return msg;
  }

  ignition::msgs::InitPointCloudPacked(msg, "world", true,
    {
      {"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32},
    });

  msg.mutable_header()->mutable_stamp()->set_sec(this->timestamps[this->timeIdx]);

  pcl::PCLPointCloud2 pclPC2;
  pcl::toPCLPointCloud2(*this->timeSpaceCoords[this->timeIdx].get(), pclPC2);

  msg.set_height(pclPC2.height);
  msg.set_width(pclPC2.width);
  msg.set_is_bigendian(pclPC2.is_bigendian);
  msg.set_point_step(pclPC2.point_step);
  msg.set_row_step(pclPC2.row_step);
  msg.set_is_dense(pclPC2.is_dense);

  msg.mutable_data()->resize(pclPC2.data.size());
  memcpy(msg.mutable_data()->data(), pclPC2.data.data(), pclPC2.data.size());

  // Populate float arrays for actual science data
  *this->tempMsg.mutable_data() = {temperatureArr[this->timeIdx].begin(),
    temperatureArr[this->timeIdx].end()};
  *this->chlorMsg.mutable_data() = {chlorophyllArr[this->timeIdx].begin(),
    chlorophyllArr[this->timeIdx].end()};
  *this->salMsg.mutable_data() = {salinityArr[this->timeIdx].begin(),
    salinityArr[this->timeIdx].end()};

  return msg;
}

IGNITION_ADD_PLUGIN(
  tethys::ScienceSensorsSystem,
  ignition::gazebo::System,
  tethys::ScienceSensorsSystem::ISystemConfigure,
  tethys::ScienceSensorsSystem::ISystemPreUpdate,
  tethys::ScienceSensorsSystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(tethys::ScienceSensorsSystem,
    "tethys::ScienceSensorsSystem")
