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

#include <gz/msgs/pointcloud_packed.pb.h>


#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/sim/World.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/VolumetricGridLookupField.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include "ScienceSensorsSystem.hh"

using namespace tethys;

class tethys::ScienceSensorsSystemPrivate
{
  /// \brief Advertise topics and services.
  public: void StartTransport();

  //////////////////////////////////
  // Functions for data manipulation

  /// \brief Called when plugin is asked to reload file.
  /// \param[in] _filepath Path to file to reload.
  public: void OnReloadData(const gz::msgs::StringMsg &_filepath)
  {
    igndbg << "Reloading file " << _filepath.data() << "\n";

    // Trigger reload and reread data
    std::lock_guard<std::mutex> lock(this->dataMutex);
    this->newDataAvailable = true;
    this->dataPath = _filepath.data();
  }

  /// \brief mutex for updating data.
  public: std::mutex dataMutex;

  /// \brief Set to true when there is a new file to be read.
  public: std::atomic<bool> newDataAvailable{true};

  /// \brief Reads csv file and populate various data fields
  /// \param[in] _ecm Immutable reference to the ECM
  public: bool ReadData(const gz::sim::EntityComponentManager &_ecm);

  //////////////////////////////
  // Functions for communication

  /// \brief Publish the latest point cloud
  public: void PublishData();

  /// \brief Service callback for a point cloud with the latest position data.
  /// \param[in] _res Point cloud to return
  /// \return True
  public: bool PointCloudService(gz::msgs::PointCloudPacked &_res);

  /// \brief Service callback for a float vector with the latest temperature data.
  /// \param[in] _res Float vector to return
  /// \return True
  public: bool TemperatureService(gz::msgs::Float_V &_res);

  /// \brief Service callback for a float vector with the latest chlorophyll data.
  /// \param[in] _res Float vector to return
  /// \return True
  public: bool ChlorophyllService(gz::msgs::Float_V &_res);

  /// \brief Service callback for a float vector with the latest salinity data.
  /// \param[in] _res Float vector to return
  /// \return True
  public: bool SalinityService(gz::msgs::Float_V &_res);

  /// \brief Returns a point cloud message populated with the latest sensor data
  public: gz::msgs::PointCloudPacked PointCloudMsg();

  /// \brief Interpolate in time between two sensor data points
  public: float InterpolateInTime(
    const gz::math::Vector3d &_point,
    const double _simTimeSeconds,
    const std::vector<std::vector<float>> &_dataArray,
    const double _tol = 1e-10);

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
  public: gz::math::Vector3d lastSensorPosENU = {
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

  //////////////////////////////////
  // Variables for data manipulation

  /// \brief Index of the latest time slice
  public: std::size_t timeIdx {0};

  /// \brief Timestamps to index slices of data
  public: std::vector<float> timestamps;

  /// \brief Spatial coordinates of data
  /// Vector size: number of time slices. Indices correspond to those of
  /// this->timestamps.
  /// Point cloud: spatial coordinates to index science data by location
  /// in the ENU world frame.
  /// TODO(arjo): remove dependence on PCL. We literally are only using it for
  /// the visuallization.
  public: std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> timeSpaceCoords;


  public: std::vector<std::vector<gz::math::Vector3d>>
    timeSpaceCoordsLatLon;

  /// \brief Spatial index of data.
  /// Vector size: number of time slices. Indices correspond to those of
  public: std::vector<gz::math::VolumetricGridLookupField<double>>
    timeSpaceIndex;

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
  public: gz::sim::World world;

  /// \brief Node for communication
  public: gz::transport::Node node;

  /// \brief Publisher for point clouds representing positions for science data
  public: gz::transport::Node::Publisher cloudPub;

  /// \brief Name used for both the point cloud topic and service
  public: std::string cloudTopic {"/science_data"};

  /// \brief Publisher for temperature
  public: gz::transport::Node::Publisher tempPub;

  /// \brief Publisher for chlorophyll
  public: gz::transport::Node::Publisher chlorPub;

  /// \brief Publisher for salinity
  public: gz::transport::Node::Publisher salPub;

  /// \brief Temperature message
  public: gz::msgs::Float_V tempMsg;

  /// \brief Chlorophyll message
  public: gz::msgs::Float_V chlorMsg;

  /// \brief Salinity message
  public: gz::msgs::Float_V salMsg;

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
    gz::sim::EntityComponentManager &_ecm,
    const gz::sim::Entity &_entity,
    const gz::sim::components::CustomSensor *_custom,
    const gz::sim::components::ParentEntity *_parent)
{
  auto type = gz::sensors::customType(_custom->Data());
  if (SensorType::kTypeStr != type)
  {
    return;
  }
  // Get sensor's scoped name without the world
  auto sensorScopedName = gz::sim::removeParentScope(
      gz::sim::scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _custom->Data();
  data.SetName(sensorScopedName);

  // Default to scoped name as topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/" + SensorType::kTypeStr;
    data.SetTopic(topic);
  }

  gz::sensors::SensorFactory sensorFactory;
  auto sensor = sensorFactory.CreateSensor<SensorType>(data);
  if (nullptr == sensor)
  {
    ignerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // Set sensor parent
  auto parentName = _ecm.Component<gz::sim::components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  // Set topic on Gazebo
  _ecm.CreateComponent(_entity,
      gz::sim::components::SensorTopic(sensor->Topic()));

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
float ScienceSensorsSystemPrivate::InterpolateInTime(
  const gz::math::Vector3d &_point,
  const double _simTimeSeconds,
  const std::vector<std::vector<float>> &_dataArray,
  const double _tol)
{
  // Get spatial interpolators for current time
  const auto& timeslice1 = this->timeSpaceIndex[this->timeIdx];
  auto interpolatorsTime1 = timeslice1.GetInterpolators(_point);

  if (interpolatorsTime1.size() == 0) return std::nanf("");
  if (!interpolatorsTime1[0].index.has_value()) return std::nanf("");

  const auto data1 = timeslice1.EstimateValueUsingTrilinear(
    interpolatorsTime1,
    _point,
    _dataArray[this->timeIdx]
  );

  if (this->timeIdx + 1 >= this->timeSpaceIndex.size())
  {
    // If we reached the end of the dataset then return the last value
    return data1.value_or(std::nanf(""));
  }

  // Get spatial interpolators for the next time
  auto nextTimeIdx = this->timeIdx + 1;
  const auto& timeslice2 = this->timeSpaceIndex[nextTimeIdx];
  auto interpolatorsTime2 = timeslice2.GetInterpolators(_point);

  if (interpolatorsTime2.size() == 0) return std::nanf("");
  if (!interpolatorsTime2[0].index.has_value()) return std::nanf("");

  const auto data2 = timeslice2.EstimateValueUsingTrilinear(
    interpolatorsTime2,
    _point,
    _dataArray[nextTimeIdx]
  );


  auto prevTimeStamp = this->timestamps[this->timeIdx];
  auto nextTimeStamp = this->timestamps[nextTimeIdx];

  auto dist = nextTimeStamp - prevTimeStamp;
  if (dist < _tol)
  {
    return data1.value_or(std::nanf(""));
  }
  else
  {
    if (data1.has_value() && data2.has_value())
    {
      return (data1.value() * (nextTimeStamp - _simTimeSeconds) +
        data2.value() * (_simTimeSeconds - prevTimeStamp)) / dist;
    }
    else
    {
      return std::nanf("");
    }
  }
}

/////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::ReadData(
    const gz::sim::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ScienceSensorsSystemPrivate::ReadData");

  if (!this->sphericalCoordinatesInitialized)
  {
    ignerr << "Trying to read data before spherical coordinates were "
           << "initialized." << std::endl;
    return false;
  }

  // Reset all data
  timeSpaceCoords.clear();
  timeSpaceCoordsLatLon.clear();
  timeSpaceIndex.clear();
  temperatureArr.clear();
  salinityArr.clear();
  chlorophyllArr.clear();
  eastCurrentArr.clear();
  northCurrentArr.clear();

  std::fstream fs;
  fs.open(this->dataPath, std::ios::in);

  if (!fs.is_open())
  {
    ignerr << "Failed to open file [" << this->dataPath << "]" << std::endl;
    return false;
  }

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
          this->timeSpaceCoordsLatLon.push_back({});
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
        auto cart =
          this->world.SphericalCoordinates(_ecm)->LocalFromSphericalPosition(
            {latitude, longitude, -depth});
        this->timeSpaceCoords[lineTimeIdx]->push_back(
          pcl::PointXYZ(cart.X(), cart.Y(), cart.Z()));
        this->timeSpaceCoordsLatLon[lineTimeIdx].emplace_back(
          latitude, longitude, depth);

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

  for(const auto &pCloud: this->timeSpaceCoordsLatLon)
  {
    this->timeSpaceIndex.emplace_back(pCloud);
  }

  // Make sure the number of timestamps in the 1D indexing array, and the
  // number of time slices of data, are the same.
  assert(this->timestamps.size() == this->timeSpaceCoords.size());

  return true;
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
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_eventMgr)
{
  this->dataPtr->world = gz::sim::World(_entity);

  if (_sdf->HasElement("data_path"))
  {
    this->dataPtr->dataPath = _sdf->Get<std::string>("data_path");
  }

  gz::common::SystemPaths sysPaths;
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

  this->dataPtr->node.Subscribe("/world/science_sensor/environment_data_path",
                                &ScienceSensorsSystemPrivate::OnReloadData,
                                this->dataPtr.get());
}

/////////////////////////////////////////////////
void ScienceSensorsSystemPrivate::StartTransport()
{
  // Advertise cloud as a service for requests on-demand, and a topic for updates
  this->cloudPub = this->node.Advertise<
      gz::msgs::PointCloudPacked>(this->cloudTopic);

  this->node.Advertise(this->cloudTopic,
      &ScienceSensorsSystemPrivate::PointCloudService, this);

  // Advertise science data, also as service and topics
  std::string temperatureTopic{"/temperature"};
  this->tempPub = this->node.Advertise<
      gz::msgs::Float_V>(temperatureTopic);
  this->node.Advertise(temperatureTopic,
      &ScienceSensorsSystemPrivate::TemperatureService, this);

  std::string chlorophyllTopic{"/chloropyll"};
  this->chlorPub = this->node.Advertise<
      gz::msgs::Float_V>(chlorophyllTopic);
  this->node.Advertise(chlorophyllTopic,
      &ScienceSensorsSystemPrivate::ChlorophyllService, this);

  std::string salinityTopic{"/salinity"};
  this->salPub = this->node.Advertise<
      gz::msgs::Float_V>(salinityTopic);
  this->node.Advertise(salinityTopic,
      &ScienceSensorsSystemPrivate::SalinityService, this);
}

/////////////////////////////////////////////////
void ScienceSensorsSystem::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        createSensor<SalinitySensor>(this, _ecm, _entity, _custom, _parent);
        createSensor<TemperatureSensor>(this, _ecm, _entity, _custom, _parent);
        createSensor<ChlorophyllSensor>(this, _ecm, _entity, _custom, _parent);
        createSensor<CurrentSensor>(this, _ecm, _entity, _custom, _parent);
        return true;
      });
}

/////////////////////////////////////////////////
void ScienceSensorsSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
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
      this->dataPtr->StartTransport();
      std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);
      this->dataPtr->ReadData(_ecm);
      this->dataPtr->newDataAvailable = false;
    }
    else
    {
      // TODO(chapulina) Throttle if it becomes spammy
      ignwarn << "Science sensor data won't be published because spherical "
              << "coordinates are unknown." << std::endl;
      return;
    }
  }
  else{
    if (this->dataPtr->newDataAvailable.load())
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);
      auto result = this->dataPtr->ReadData(_ecm);
      this->dataPtr->newDataAvailable = !result;
    }
  }

  double simTimeSeconds = std::chrono::duration<double>(
    _info.simTime).count();

  if(this->dataPtr->timeIdx + 1 < this->dataPtr->timestamps.size())
  {
    if(simTimeSeconds >= this->dataPtr->timestamps[this->dataPtr->timeIdx + 1])
    {
      this->dataPtr->timeIdx++;
    }
  }

  // Publish every n iters so that GUI PointCloud plugin gets it.
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

  // Sensor position to interpolate for
  gz::math::Vector3d sensorPosENU;


  // For each sensor, interpolate using existing data at neighboring positions,
  // to generate data for that sensor.
  for (auto &[entity, sensor] : this->entitySensorMap)
  {
    IGN_PROFILE("ScienceSensorsSystem::LookupInterpolators");
    auto sensorPosENU = gz::sim::worldPose(entity, _ecm).Pos();
    auto spherical = gz::sim::sphericalCoordinates(entity, _ecm).value();
    auto sphericalDepthCorrected = gz::math::Vector3d{spherical.X(), spherical.Y(),
      -spherical.Z()};

    if (auto casted = std::dynamic_pointer_cast<SalinitySensor>(sensor))
    {
      casted->SetData(
        this->dataPtr->InterpolateInTime(
          sphericalDepthCorrected, simTimeSeconds, this->dataPtr->salinityArr));
    }
    else if (auto casted = std::dynamic_pointer_cast<TemperatureSensor>(
      sensor))
    {
      const auto temp = this->dataPtr->InterpolateInTime(
        sphericalDepthCorrected, simTimeSeconds, this->dataPtr->temperatureArr);

      gz::math::Temperature tempC;
      tempC.SetCelsius(temp);
      casted->SetData(tempC);
    }
    else if (auto casted = std::dynamic_pointer_cast<ChlorophyllSensor>(
      sensor))
    {
      casted->SetData(
        this->dataPtr->InterpolateInTime(
          sphericalDepthCorrected, simTimeSeconds,
          this->dataPtr->chlorophyllArr));
    }
    else if (auto casted = std::dynamic_pointer_cast<CurrentSensor>(
      sensor))
    {
      const auto nCurr = this->dataPtr->InterpolateInTime(
        sphericalDepthCorrected, simTimeSeconds, this->dataPtr->northCurrentArr);

      const auto eCurr = this->dataPtr->InterpolateInTime(
        sphericalDepthCorrected, simTimeSeconds, this->dataPtr->eastCurrentArr);


      gz::math::Vector3d current(nCurr, eCurr, 0);
      casted->SetData(current);
    }
    else
    {
      ignerr << "Unsupported sensor type, failed to set data" << std::endl;
    }

    // Update all the sensors
    sensor->Update(_info.simTime, false);
  }
}

//////////////////////////////////////////////////
void ScienceSensorsSystem::RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ScienceSensorsSystem::RemoveSensorEntities");

  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *)->bool
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
    gz::msgs::PointCloudPacked &_res)
{
  _res = this->PointCloudMsg();
  return true;
}

//////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::TemperatureService(
    gz::msgs::Float_V &_res)
{
  _res = this->tempMsg;
  return true;
}

//////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::ChlorophyllService(
    gz::msgs::Float_V &_res)
{
  _res = this->chlorMsg;
  return true;
}

//////////////////////////////////////////////////
bool ScienceSensorsSystemPrivate::SalinityService(
    gz::msgs::Float_V &_res)
{
  _res = this->salMsg;
  return true;
}

//////////////////////////////////////////////////
gz::msgs::PointCloudPacked ScienceSensorsSystemPrivate::PointCloudMsg()
{
  IGN_PROFILE("ScienceSensorsSystemPrivate::PointCloudMsg");

  gz::msgs::PointCloudPacked msg;

  if (this->timeIdx < 0 || this->timeIdx >= this->timestamps.size())
  {
    ignerr << "Invalid time index [" << this->timeIdx << "]."
           << std::endl;
    return msg;
  }

  gz::msgs::InitPointCloudPacked(msg, "world", true,
    {
      {"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
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
  gz::sim::System,
  tethys::ScienceSensorsSystem::ISystemConfigure,
  tethys::ScienceSensorsSystem::ISystemPreUpdate,
  tethys::ScienceSensorsSystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(tethys::ScienceSensorsSystem,
    "tethys::ScienceSensorsSystem")