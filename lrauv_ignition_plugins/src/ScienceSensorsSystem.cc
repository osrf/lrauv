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
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include "ScienceSensorsSystem.hh"

using namespace tethys;

class tethys::ScienceSensorsSystemPrivate
{
  //////////////////////////////////
  // Functions for data manipulation

  /// \brief Reads csv file and populate various data fields
  /// \param[in] _ecm Immutable reference to the ECM
  public: void ReadData(const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Generate octree from spatial data, for searching
  public: void GenerateOctrees();

  /// \brief Barycentric interpolation in 3D, given 4 points on any arbitrary
  /// tetrahedra.
  /// \param[in] _p Position within the tetrahedra to interpolate for
  /// \param[in] _xyzs XYZ coordinates of 4 vertices of a tetrahedra
  /// \param[in] _values Data values at the 4 vertices
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float BarycentricInterpolate(
    const Eigen::Vector3f &_p,
    const std::vector<pcl::PointXYZ> &_xyzs,
    const std::vector<float> &_values);

  /// \brief Barycentric interpolation in 2D, given 3 points on any arbitrary
  /// triangle.
  /// \param[in] _p Position within the triangle to interpolate for
  /// \param[in] _xyzs XYZ coordinates of 3 vertices of a triangle
  /// \param[in] _values Data values at the 3 vertices
  /// \return Interpolated value, or quiet NaN if inputs invalid.
  public: float BarycentricInterpolate(
    const Eigen::Vector2f &_p,
    const std::vector<pcl::PointXYZ> &_xyzs,
    const std::vector<float> &_values);

  /// \brief Extract elements at indices _inds from _orig vector.
  /// \param[in] _orig A vector of values to extract
  /// \param[in] _inds Indices of elements to extract
  /// \param[out] _new Extracted values
  public: void ExtractElements(
    const std::vector<float> &_orig,
    const std::vector<int> &_inds,
    std::vector<float> &_new);

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

  //////////////////////////////
  // Constants for visualization

  // TODO This is a workaround pending upstream Ignition orbit tool improvements
  // \brief Scale down in order to see in view
  // For 2003080103_mb_l3_las_1x1km.csv
  //public: const float MINIATURE_SCALE = 0.01;
  // For 2003080103_mb_l3_las.csv
  public: const float MINIATURE_SCALE = 0.0001;
  // For simple_test.csv
  //public: const float MINIATURE_SCALE = 1.0;

  // TODO This is a workaround pending upstream Marker performance improvements.
  // \brief Performance trick. Skip depths below this z, so have memory to
  // visualize higher layers at higher resolution.
  // This is only for visualization, so that MAX_PTS_VIS can calculate close
  // to the actual number of points visualized.
  // Sensors shouldn't use this.
  public: const float SKIP_Z_BELOW = -20;
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
        // Performance trick. Skip points below a certain depth
        if (-depth < this->SKIP_Z_BELOW)
        {
          continue;
        }

        // Convert lat / lon / elevation to Cartesian ENU
        auto cart = this->world.SphericalCoordinates(_ecm).value()
            .PositionTransform({IGN_DTOR(latitude), IGN_DTOR(longitude), 0.0},
            ignition::math::SphericalCoordinates::SPHERICAL,
            ignition::math::SphericalCoordinates::LOCAL2);
        // Flip sign of z, because positive depth is negative z.
        cart.Z() = -depth;

        // Performance trick. Scale down to see in view
        cart *= this->MINIATURE_SCALE;
        // Revert Z to the unscaled depth
        cart.Z() = -depth;

        // Performance trick. Skip points beyond some distance from origin
        if (abs(cart.X()) > 1000 || abs(cart.Y()) > 1000)
        {
          continue;
        }

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
float ScienceSensorsSystemPrivate::BarycentricInterpolate(
  const Eigen::Vector3f &_p,
  const std::vector<pcl::PointXYZ> &_xyzs,
  const std::vector<float> &_values)
{
  // Implemented from https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Barycentric_coordinates_on_tetrahedra

  //igndbg << "_p: " << std::endl << _p << std::endl;

  Eigen::Matrix3f T;
  // Row 1: x1 - x4, x2 - x4, x3 - x4
  T << _xyzs.at(0).x - _xyzs.at(3).x,
       _xyzs.at(1).x - _xyzs.at(3).x,
       _xyzs.at(2).x - _xyzs.at(3).x,
  // Row 2: y1 - y4, y2 - y4, y3 - y4
       _xyzs.at(0).y - _xyzs.at(3).y,
       _xyzs.at(1).y - _xyzs.at(3).y,
       _xyzs.at(2).y - _xyzs.at(3).y,
  // Row 3: z1 - z4, z2 - z4, z3 - z4
       _xyzs.at(0).z - _xyzs.at(3).z,
       _xyzs.at(1).z - _xyzs.at(3).z,
       _xyzs.at(2).z - _xyzs.at(3).z;
  //igndbg << "T: " << std::endl << T << std::endl;

  // Check for special case when 3rd row of T is all zeros.
  // Happens when all neighbors are on the same z slice
  if ((T.row(2).array() < 1e-6).all())
  {
    igndbg << "4 points on same z slice. Using 2D barycentric interpolation."
      << std::endl;
    Eigen::Vector2f p2D;
    p2D << _p(0), _p(1);
    return this->BarycentricInterpolate(p2D, _xyzs, _values);
  }

  // r4 = (x4, y4, z4)
  Eigen::Vector3f r4;
  r4 << _xyzs.at(3).x, _xyzs.at(3).y, _xyzs.at(3).z;
  //igndbg << "r4: " << std::endl << r4 << std::endl;

  // (lambda1, lambda2, lambda3)
  Eigen::Vector3f lambda123 = T.inverse() * (_p - r4);

  //igndbg << "T.inverse(): " << std::endl << T.inverse() << std::endl;

  // lambda4 = 1 - lambda1 - lambda2 - lambda3
  float lambda4 = 1 - lambda123(0) - lambda123(1) - lambda123(2);

  //igndbg << "Barycentric 3D lambda 1 2 3 4: " << lambda123(0) << ", "
  //  << lambda123(1) << ", "
  //  << lambda123(2) << ", "
  //  << lambda4 << std::endl;

  // f(r) = lambda1 * f(r1) + lambda2 * f(r2) + lambda3 * f(r3)
  float result =
    lambda123(0) * _values[0] +
    lambda123(1) * _values[1] +
    lambda123(2) * _values[2] +
    lambda4 * _values[3];

  igndbg << "Barycentric 3D interpolation of values " << _values[0] << ", "
    << _values[1] << ", " << _values[2] << ", " << _values[3]
    << " resulted in " << result << std::endl;

  return result;
}

/////////////////////////////////////////////////
float ScienceSensorsSystemPrivate::BarycentricInterpolate(
  const Eigen::Vector2f &_p,
  const std::vector<pcl::PointXYZ> &_xyzs,
  const std::vector<float> &_values)
{
  // 2D case, consider inputs a triangle and use 2 x 2 matrix for T
  Eigen::Matrix2f T;
  // Row 1: x1 - x3, x2 - x3
  T << _xyzs.at(0).x - _xyzs.at(2).x,
       _xyzs.at(1).x - _xyzs.at(2).x,
  // Row 2: y1 - y3, y2 - y3
       _xyzs.at(0).y - _xyzs.at(2).y,
       _xyzs.at(1).y - _xyzs.at(2).y;
  //igndbg << "T: " << std::endl << T << std::endl;

  // r3 = (x3, y3, z3)
  Eigen::Vector2f r3;
  r3 << _xyzs.at(2).x, _xyzs.at(2).y;
  //igndbg << "r3: " << std::endl << r3 << std::endl;

  // (lambda1, lambda2)
  Eigen::Vector2f lambda12 = T.inverse() * (_p - r3);

  //igndbg << "T.inverse(): " << std::endl << T.inverse() << std::endl;

  // lambda3 = 1 - lambda1 - lambda2
  float lambda3 = 1 - lambda12(0) - lambda12(1);

  //igndbg << "Barycentric 2D lambda 1 2 3: " << lambda12(0) << ", "
  //  << lambda12(1) << ", "
  //  << lambda3 << std::endl;

  // f(r) = lambda1 * f(r1) + lambda2 * f(r2)
  float result =
    lambda12(0) * _values[0] +
    lambda12(1) * _values[1] +
    lambda3 * _values[2];

  igndbg << "Barycentric 2D interpolation of values " << _values[0] << ", "
    << _values[1] << ", " << _values[2]
    << " resulted in " << result << std::endl;

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

  // Get a sensor's pose, search in the octree for the closest neighbors,
  // and interpolate to get approximate data at this sensor pose.
  // Only need to done for one sensor. All sensors are on the robot, doesn't
  // make a big difference to data location.
  for (auto &[entity, sensor] : this->entitySensorMap)
  {
    // Sensor pose in ENU, used to search for data by spatial coordinates
    auto sensorPosENU = ignition::gazebo::worldPose(entity, _ecm).Pos();
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

    igndbg << "Searching around sensor Cartesian location "
      << std::round(searchPoint.x * 1000.0) / 1000.0 << ", "
      << std::round(searchPoint.y * 1000.0) / 1000.0 << ", "
      << std::round(searchPoint.z * 1000.0) / 1000.0 << std::endl;

    // Indices and distances of neighboring points in the search results
    std::vector<int> spatialIdx;
    std::vector<float> spatialSqrDist;

    // If there are any nodes in the octree, search in octree to find spatial
    // index of science data
    if (this->dataPtr->spatialOctrees[this->dataPtr->timeIdx]->getLeafCount()
      > 0)
    {
      IGN_PROFILE("ScienceSensorsSystem::PostUpdate nearestKSearch");

      // kNN search (alternatives are voxel search and radius search. kNN
      // search is good for variable resolution when the distance to the next
      // neighbor is unknown).
      // Find 4 nearest neighbors for barycentric interpolation
      if (this->dataPtr->spatialOctrees[this->dataPtr->timeIdx]
        ->nearestKSearch(searchPoint, 4, spatialIdx, spatialSqrDist) <= 0)
      {
        ignwarn << "Not enough data found near sensor location " << sensorPosENU
          << std::endl;
        continue;
      }
      // Debug output
      else
      {
        for (std::size_t i = 0; i < spatialIdx.size(); i++)
        {
          // Index the point cloud at the current time slice
          pcl::PointXYZ nbrPt = this->dataPtr->timeSpaceCoords[
            this->dataPtr->timeIdx]->at(spatialIdx[i]);

          igndbg << "Neighbor at ("
            << std::round(nbrPt.x * 1000) / 1000.0 << ", "
            << std::round(nbrPt.y * 1000) / 1000.0 << ", "
            << std::round(nbrPt.z * 1000) / 1000.0
            << "), squared distance " << spatialSqrDist[i]
            << " m" << std::endl;
        }
      }

      // Get neighbor XYZs to pass to barycentric interpolation
      std::vector<pcl::PointXYZ> interpolatorXYZs;
      for (int i = 0; i < spatialIdx.size(); ++i)
      {
        interpolatorXYZs.push_back(this->dataPtr->timeSpaceCoords[
          this->dataPtr->timeIdx]->at(spatialIdx[i]));
      }

      // Convert to Eigen to pass to interpolation
      Eigen::Vector3f sensorPosENUEigen;
      sensorPosENUEigen << sensorPosENU.X(), sensorPosENU.Y(), sensorPosENU.Z();

      // Input values to barycentric interpolation
      std::vector<float> interpolationValues;

      // For the correct sensor, interpolate using nearby locations with data
      if (auto casted = std::dynamic_pointer_cast<SalinitySensor>(sensor))
      {
        igndbg << "Interpolating salinity" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->salinityArr[this->dataPtr->timeIdx], spatialIdx,
          interpolationValues);
        float sal = this->dataPtr->BarycentricInterpolate(
          sensorPosENUEigen, interpolatorXYZs, interpolationValues);
        casted->SetData(sal);
      }
      else if (auto casted = std::dynamic_pointer_cast<TemperatureSensor>(
        sensor))
      {
        igndbg << "Interpolating temperature" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->temperatureArr[this->dataPtr->timeIdx], spatialIdx,
          interpolationValues);
        float temp = this->dataPtr->BarycentricInterpolate(
          sensorPosENUEigen, interpolatorXYZs, interpolationValues);

        ignition::math::Temperature tempC;
        tempC.SetCelsius(temp);
        casted->SetData(tempC);
      }
      else if (auto casted = std::dynamic_pointer_cast<ChlorophyllSensor>(
        sensor))
      {
        igndbg << "Interpolating chlorophyll" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->chlorophyllArr[this->dataPtr->timeIdx], spatialIdx,
          interpolationValues);
        float chlor = this->dataPtr->BarycentricInterpolate(
          sensorPosENUEigen, interpolatorXYZs, interpolationValues);
        casted->SetData(chlor);
      }
      else if (auto casted = std::dynamic_pointer_cast<CurrentSensor>(
        sensor))
      {
        igndbg << "Interpolating E and N currents" << std::endl;
        this->dataPtr->ExtractElements(
          this->dataPtr->eastCurrentArr[this->dataPtr->timeIdx], spatialIdx,
          interpolationValues);
        float eCurr = this->dataPtr->BarycentricInterpolate(
          sensorPosENUEigen, interpolatorXYZs, interpolationValues);

        // Reset before reuse
        interpolationValues.clear();

        this->dataPtr->ExtractElements(
          this->dataPtr->northCurrentArr[this->dataPtr->timeIdx], spatialIdx,
          interpolationValues);
        float nCurr = this->dataPtr->BarycentricInterpolate(
          sensorPosENUEigen, interpolatorXYZs, interpolationValues);

        auto curr = ignition::math::Vector3d(eCurr, nCurr, 0.0);
        casted->SetData(curr);
      }
      else
      {
        ignerr << "Unsupported sensor type, failed to set data" << std::endl;
      }
    }

    // Update last update position to the current position
    this->dataPtr->lastSensorPosENU = sensorPosENU;

    // Only need to find position ONCE for the entire robot. Don't need to
    // repeat for every sensor.
    break;
  }

  // Update all the sensors
  for (auto &[entity, sensor] : this->entitySensorMap)
  {
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

  // TODO optimization for visualization:
  // Use PCL methods to chop off points beyond some distance from sensor
  // pose. Don't need to visualize beyond that. Might want to put that on a
  // different topic specifically for visualization.

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
