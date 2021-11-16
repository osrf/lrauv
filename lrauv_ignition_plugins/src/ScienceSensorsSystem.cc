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

#include <ignition/msgs/pointcloud_packed.pb.h>

#include <ignition/common/SystemPaths.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/octree/octree_search.h>

#include "ScienceSensorsSystem.hh"

using namespace tethys;

class tethys::ScienceSensorsSystemPrivate
{
  /// \brief Reads csv file and populate various data fields
  public: void ReadData();

  /// \brief Generate octree from spatial data, for searching
  public: void GenerateOctrees();

  /// \brief Find XYZ locations of points in the two closest z slices to
  /// interpolate among.
  /// \param[in] _pt Location in space to interpolate for
  /// \param[in] _inds Indices in _arr
  /// \param[in] _sqrDists Distances of elements in _arr
  /// \param[out] _interpolatorIndsRV Result. Indices of points to interpolate
  /// among
  public: void FindInterpolators(
    pcl::PointXYZ &_pt,
    std::vector<int> &_inds,
    std::vector<float> &_sqrDists,
    std::vector<int> &_interpolatorIndsRV);

  /// \brief Create a slice from a point cloud, represented in a matrix of size
  /// 3 x n, of points sharing the same given z value.
  /// \param[in] _searchPt Location in space to interpolate for
  /// \param[in] _depth Target z value
  /// \param[in] _points Points in the original point cloud
  /// \param[out] _zSlice Points in the new point cloud slice at _depth
  /// \param[out] _zSliceInds Indices of points in _zSlices in the original
  /// point cloud _points
  /// \param[out] _interpolatorInds Result of octree search, indices of points.
  /// \param[out] _interpolatorSqrDists Result of octree search, distances.
  public: void CreateDepthSlice(
    pcl::PointXYZ &_searchPt,
    float _depth,
    Eigen::MatrixXf &_points,
    pcl::PointCloud<pcl::PointXYZ> &_zSlice,
    std::vector<int> &_zSliceInds,
    std::vector<int> &_interpolatorInds,
    std::vector<float> &_interpolatorSqrDists);

  /// \brief Publish the latest point cloud
  public: void PublishData();

  /// \brief Service that provides the latest science data
  public: bool ScienceDataService(ignition::msgs::PointCloudPacked &_res);

  /// \brief Returns a point cloud message populated with the latest sensor data
  public: ignition::msgs::PointCloudPacked PointCloudMsg();

  /// \brief Interpolate floating point data based on distance
  /// \param[in] _arr Array of data from which to find elements to interpolate
  /// \param[in] _inds Indices in _arr
  /// \param[in] _sqrDists Distances of elements in _arr
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float InterpolateData(
    std::vector<float> _arr,
    std::vector<int> &_inds,
    std::vector<float> &_sqrDists);

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

  /// \brief Input data file name, relative to a path Ignition can find in its
  /// environment variables.
  public: std::string dataPath {"2003080103_mb_l3_las.csv"};

  /// \brief Indicates whether data has been loaded
  public: bool initialized = false;

  /// \brief Set to true after the spherical coordinates have been initialized.
  /// This may happen at startup if the SDF file has them hardcoded, or at
  /// runtime when the first vehicle is spawned. Assume the coordinates are
  /// only shifted once.
  public: bool worldSphericalCoordsInitialized {false};

  /// \brief Whether using more than one time slices of data
  public: bool multipleTimeSlices = false;

  /// \brief Index of the latest time slice
  public: int timeIdx = 0;

  /// \brief Timestamps to index slices of data
  public: std::vector<float> timestamps;

  /// \brief Spatial coordinates of data
  /// Vector size: number of time slices. Indices correspond to those of
  /// this->timestamps.
  /// Point cloud: spatial coordinates to index science data by location
  public: std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> timeSpaceCoords;

  /// \brief Resolution of spatial coordinates in meters in octree, for data
  /// search.
  public: float spatialRes = 0.1f;

  /// \brief Octree for data search based on spatial location of sensor. One
  /// octree per point cloud in timeSpaceCoords.
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

  /// \brief World object to access world properties.
  public: ignition::gazebo::World world;

  /// \brief Node for communication
  public: ignition::transport::Node node;

  /// \brief Publisher for point cloud data
  public: ignition::transport::Node::Publisher cloudPub;
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
void ScienceSensorsSystemPrivate::ReadData()
{
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
        // Gather spatial coordinates, 3 fields in the line, into point cloud
        // for indexing this time slice of data.
        this->timeSpaceCoords[lineTimeIdx]->push_back(
          pcl::PointXYZ(latitude, longitude, depth));

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

  this->initialized = true;
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
void ScienceSensorsSystemPrivate::FindInterpolators(
  pcl::PointXYZ &_pt,
  std::vector<int> &_inds,
  std::vector<float> &_sqrDists,
  std::vector<int> &_interpolatorIndsRV)
{
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
    ignwarn << "FindInterpolators(): Invalid neighbors aray size ("
            << _inds.size() << " and " << _sqrDists.size()
            << "). No neighbors to use for interpolation. Returning NaN."
            << std::endl;
    return;
  }
  if (_inds.size() != _sqrDists.size())
  {
    ignwarn << "FindInterpolators(): Number of neighbors != number of "
            << "distances. Invalid input. Returning NaN." << std::endl;
    return;
  }

  // Closest neighbor
  // The distances from kNN search are sorted, so can always just take [0]th
  int nnIdx = _inds[0];
  float minDist = _sqrDists[0];
  // Get z of neighbor
  float nnZ = this->timeSpaceCoords[this->timeIdx]->at(nnIdx).z;

  // TODO: Remove all points at the z of the closest neighbor, so the second
  // closest neighbor will fall into another slice

  // Second-closest neighbor
  int nnIdx2 = _inds[1];
  float minDist2 = _sqrDists[1];
  // Get z of neighbor
  float nnZ2 = this->timeSpaceCoords[this->timeIdx]->at(nnIdx2).z;

  // Filter out just the slice of point cloud at the two closest z values

  // Get raw Eigen matrix of all spatial locations. Take only the block of
  // the matrix containing the z values of the points.
  // Matrix size 3 x n
  Eigen::MatrixXf points =
    this->timeSpaceCoords[this->timeIdx]->getMatrixXfMap(3, 4, 0).block(
      0, 0, 3, this->timeSpaceCoords[this->timeIdx]->size());

  //Eigen::MatrixXf zSlice1, zSlice2;
  pcl::PointCloud<pcl::PointXYZ> zSlice1, zSlice2;
  std::vector<int> zSliceInds1, zSliceInds2;

  // Indices and distances of neighboring points in the search results
  // 4 above, 4 below
  std::vector<int> interpolatorInds1, interpolatorInds2;
  std::vector<float> interpolatorSqrDists1, interpolatorSqrDists2;

  igndbg << this->timeSpaceCoords[this->timeIdx]->size() << " points"
    << std::endl;
  igndbg << "Slice 1 nnIdx " << nnIdx << ", dist " << sqrt(minDist)
    << std::endl;
  this->CreateDepthSlice(_pt, nnZ, points, zSlice1, zSliceInds1,
    interpolatorInds1, interpolatorSqrDists1);

  igndbg << "Slice 2 nnIdx " << nnIdx2 << ", dist " << sqrt(minDist2)
    << std::endl;
  this->CreateDepthSlice(_pt, nnZ2, points, zSlice2, zSliceInds2,
    interpolatorInds2, interpolatorSqrDists2);

  // TODO: Return indices of 8 points in original point cloud, using zSliceInds
  //interpolatorIndsRV
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::CreateDepthSlice(
  pcl::PointXYZ &_searchPt,
  float _depth,
  Eigen::MatrixXf &_points,
  pcl::PointCloud<pcl::PointXYZ> &_zSlice,
  std::vector<int> &_zSliceInds,
  std::vector<int> &_interpolatorInds,
  std::vector<float> &_interpolatorSqrDists)
{
  // Find points with z matching the closest z. These points are on
  // the same z slice
  Eigen::Matrix<bool, 1, Eigen::Dynamic> depthInds =
    (_points.row(2).array() == _depth);

  igndbg << "Number of points matching z == " << _depth << ": "
         << depthInds.count() << std::endl;

  // Logical indexing. Filter out just the points on the z slice
  // Allocate only the number of points in the slice
  //_zSlice = Eigen::MatrixXf::Zero(3, depthInds.count());
  //_zSlice = pcl::PointCloud<pcl::PointXYZ>::Ptr(
  //  new pcl::PointCloud<pcl::PointXYZ>());
  _zSlice = pcl::PointCloud<pcl::PointXYZ>();
  _zSlice.points.resize(depthInds.count());

  _zSliceInds.clear();

  int zSliceIdx = 0;
  // Go through the Boolean array
  for (int i = 0; i < depthInds.size(); ++i)
  {
    // Only look at points with z matching closest z
    if (depthInds[i])
    {
      // Keep track of new points' indices in the original point cloud
      _zSliceInds.push_back(i);

      // Retrieve the corresponding point in the original point cloud
      //_zSlice.block<3, 1>(0, zSliceIdx) = _points.block<3, 1>(0, i);

      _zSlice.points[zSliceIdx] = pcl::PointXYZ(
        _points(0, i),
        _points(1, i),
        _points(2, i));

      //igndbg << "Filtered point: " << _points(0, i) << ","
      //                             << _points(1, i) << ","
      //                             << _points(2, i) << std::endl;
      zSliceIdx++;
    }
  }

  igndbg << "Number of points in this z slice: " << zSliceIdx << std::endl;
  assert(zSliceIdx == depthInds.count());

  // Create octree for this depth slice
  auto octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(
    this->spatialRes);
  octree.setInputCloud(_zSlice.makeShared());
  octree.addPointsFromInputCloud();

  int k = 4;

  // TODO debug seg fault
  // Search in the depth slice to find 4 closest neighbors
  if (octree.nearestKSearch(
    _searchPt, k, _interpolatorInds, _interpolatorSqrDists) <= 0)
  {
    ignwarn << "No data found in slice near search point " << _searchPt
            << std::endl;
    return;
  }
  // Debug output
  else
  {
    igndbg << "Found " << _interpolatorInds.size()
           << " neighbors in this slice." << std::endl;

    for (std::size_t i = 0; i < _interpolatorInds.size(); ++i)
    {
      pcl::PointXYZ nbrPt = _zSlice.at(_interpolatorInds[i]);

      igndbg << "Neighbor at (" << nbrPt.x << ", " << nbrPt.y << ", "
             << nbrPt.z << "), squared distance " << _interpolatorSqrDists[i]
             << " m" << std::endl;
    }
  }
  //
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::PublishData()
{
  this->cloudPub.Publish(this->PointCloudMsg());
}

/////////////////////////////////////////////////
float ScienceSensorsSystemPrivate::InterpolateData(
  std::vector<float> _arr,
  std::vector<int> &_inds,
  std::vector<float> &_sqrDists)
{
  // Sanity checks
  if (_inds.size() == 0 || _sqrDists.size() == 0)
  {
    ignwarn << "InterpolateData(): Invalid neighbors aray size ("
            << _inds.size() << " and " << _sqrDists.size()
            << "). No neighbors to use for interpolation. Returning NaN."
            << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }
  if (_inds.size() != _sqrDists.size())
  {
    ignwarn << "InterpolateData(): Number of neighbors != number of distances."
            << "Invalid input. Returning NaN." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Find closest neighbor
  int nnIdx = _inds[0];
  float minDist = _sqrDists[0];
  for (int i = 0; i < _inds.size(); ++i)
  {
    if (_sqrDists[i] < minDist)
    {
      nnIdx = _inds[i];
      minDist = _sqrDists[i];
    }
  }

  // Find second-closest neighbor
  int secondNnIdx = _inds[0];
  float secondMinDist = _sqrDists[secondNnIdx];
  for (int i = 0; i < _inds.size(); ++i)
  {
    if (_sqrDists[_inds[i]] < secondMinDist && i != nnIdx)
    {
      secondNnIdx = _inds[i];
      secondMinDist = _sqrDists[secondNnIdx];
    }
  }

  // Interpolate between closest and second-closest neighbors





  // Dummy: Just return the closest element
  return _arr[nnIdx];

  // TODO Return a weighted sum, based on distance, of the elements to
  // interpolate among
  // TODO: Look at x and y only? Depth resolution much finer and tips kNN
  // search. Or use radiusSearch().



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

  this->dataPtr->cloudPub = this->dataPtr->node.Advertise<
      ignition::msgs::PointCloudPacked>("/science_data");

  this->dataPtr->node.Advertise("/science_data",
      &ScienceSensorsSystemPrivate::ScienceDataService, this->dataPtr.get());

  this->dataPtr->ReadData();
  this->dataPtr->GenerateOctrees();
  this->dataPtr->PublishData();
}

/////////////////////////////////////////////////
void ScienceSensorsSystem::PreUpdate(
  const ignition::gazebo::UpdateInfo &,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->worldSphericalCoordsInitialized)
  {
    auto latLon = this->dataPtr->world.SphericalCoordinates(_ecm);
    if (latLon)
    {
      ignwarn << "New spherical coordinates detected." << std::endl;
      //TODO(chapulina) Shift science data to new coordinates
      this->dataPtr->worldSphericalCoordsInitialized = true;
    }
  }

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
  // Only update and publish if data has been loaded and simulation is not
  // paused.
  if (this->dataPtr->initialized && !_info.paused)
  {
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
        this->dataPtr->PublishData();
      }
    }

    // For each sensor, get its pose, search in the octree for the closest
    // neighbors, and interpolate to get approximate data at this sensor pose.
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      // Sensor pose in lat/lon, used to search for data by spatial coordinates
      // TODO: Convert to Cartesian!!!
      auto sensorLatLon = ignition::gazebo::sphericalCoordinates(entity, _ecm);
      if (!sensorLatLon)
      {
        static std::unordered_set<ignition::gazebo::Entity> warnedEntities;
        if (warnedEntities.find(entity) != warnedEntities.end())
        {
          ignwarn << "Failed to get spherical coordinates for sensor entity ["
                  << entity << "]" << std::endl;
          warnedEntities.insert(entity);
        }
        continue;
      }
      pcl::PointXYZ searchPoint(
          sensorLatLon.value().X(),
          sensorLatLon.value().Y(),
          sensorLatLon.value().Z());

      igndbg << "Searching around sensor location "
        << searchPoint.x << ", "
        << searchPoint.y << ", "
        << searchPoint.z << std::endl;

      // kNN search (alternatives are voxel search and radius search. kNN
      // search is good for variable resolution when the distance to the next
      // neighbor is unknown).
      int k = 8;

      // Indices and distances of neighboring points in the search results
      std::vector<int> spatialIdx;
      std::vector<float> spatialSqrDist;

      // Search in octree to find spatial index of science data
      if (this->dataPtr->spatialOctrees[this->dataPtr->timeIdx]->nearestKSearch(
        searchPoint, k, spatialIdx, spatialSqrDist) <= 0)
      {
        ignwarn << "No data found near sensor location " << sensorLatLon.value()
                << std::endl;
        continue;
      }
      // Debug output
      /*
      else
      {
        igndbg << "kNN search for sensor pose (" << sensorPose.X() << ", "
               << sensorPose.Y() << ", " << sensorPose.Z() << "):"
               << std::endl;

        for (std::size_t i = 0; i < spatialIdx.size(); i++)
        {
          // Index the point cloud at the current time slice
          pcl::PointXYZ nbrPt = (*(this->dataPtr->timeSpaceCoords[
            this->dataPtr->timeIdx]))[spatialIdx[i]];

          igndbg << "Neighbor at (" << nbrPt.x << ", " << nbrPt.y << ", "
                 << nbrPt.z << "), squared distance " << spatialSqrDist[i]
                 << " m" << std::endl;
        }
      }
      */

      std::vector<int> interpolatorInds;
      this->dataPtr->FindInterpolators(searchPoint, spatialIdx, spatialSqrDist,
        interpolatorInds);

      // TODO trilinear interpolation using the 8 interpolators found

      // For the correct sensor, grab closest neighbors and interpolate
      if (auto casted = std::dynamic_pointer_cast<SalinitySensor>(sensor))
      {
        float sal = this->dataPtr->InterpolateData(
          this->dataPtr->salinityArr[this->dataPtr->timeIdx], spatialIdx,
          spatialSqrDist);
        casted->SetData(sal);
      }
      else if (auto casted = std::dynamic_pointer_cast<TemperatureSensor>(
        sensor))
      {
        float temp = this->dataPtr->InterpolateData(
          this->dataPtr->temperatureArr[this->dataPtr->timeIdx], spatialIdx,
          spatialSqrDist);

        ignition::math::Temperature tempC;
        tempC.SetCelsius(temp);
        casted->SetData(tempC);
      }
      else if (auto casted = std::dynamic_pointer_cast<ChlorophyllSensor>(
        sensor))
      {
        float chlor = this->dataPtr->InterpolateData(
          this->dataPtr->chlorophyllArr[this->dataPtr->timeIdx], spatialIdx,
          spatialSqrDist);
        casted->SetData(chlor);
      }
      else if (auto casted = std::dynamic_pointer_cast<CurrentSensor>(sensor))
      {
        float eCurr = this->dataPtr->InterpolateData(
          this->dataPtr->eastCurrentArr[this->dataPtr->timeIdx], spatialIdx,
          spatialSqrDist);
        float nCurr = this->dataPtr->InterpolateData(
          this->dataPtr->northCurrentArr[this->dataPtr->timeIdx], spatialIdx,
          spatialSqrDist);

        auto curr = ignition::math::Vector3d(eCurr, nCurr, 0.0);
        casted->SetData(curr);
      }
      else
      {
        ignerr << "Unsupported sensor type, failed to set data" << std::endl;
      }
      sensor->Update(_info.simTime, false);
    }
  }

  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void ScienceSensorsSystem::RemoveSensorEntities(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
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
bool ScienceSensorsSystemPrivate::ScienceDataService(
    ignition::msgs::PointCloudPacked &_res)
{
  _res = this->PointCloudMsg();
  return true;
}

//////////////////////////////////////////////////
ignition::msgs::PointCloudPacked ScienceSensorsSystemPrivate::PointCloudMsg()
{
  ignition::msgs::PointCloudPacked msg;

  if (this->timeIdx < 0 || this->timeIdx >= this->timestamps.size())
  {
    ignerr << "Invalid time index [" << this->timeIdx << "]."
           << std::endl;
    return msg;
  }

  ignition::msgs::InitPointCloudPacked(msg, "world", true,
    {
      {"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32}
      // {"salinity", ignition::msgs::PointCloudPacked::Field::FLOAT32}
      // TODO(louise) Add more fields
      // https://github.com/osrf/lrauv/issues/85
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

  // FIXME: Include more than position data
  msg.mutable_data()->resize(pclPC2.data.size());
  memcpy(msg.mutable_data()->data(), pclPC2.data.data(), pclPC2.data.size());

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
