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

#include "ignition/msgs/pointcloud_packed.pb.h"

#include <string>
#include <utility>
#include <vector>

#include "PointCloudIterator.hh"
#include "VisualizePointCloud.hh"

#include <ignition/common/Console.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

namespace tethys
{
  /// \brief Private data class for VisualizePointCloud
  class VisualizePointCloudPrivate
  {
    /// \brief Transport node
    public: ignition::transport::Node node;

    /// \brief Science data type-specific topic name to subscribe
    public: std::string topicName{""};

    /// \brief List of science data topics
    public: QStringList topicList;

    /// \brief Protect variables changed from transport and the user
    public: std::recursive_mutex mutex;

    /// \brief Generic point cloud topic name
    public: std::string pcTopic = {"/science_data"};

    /// \brief Generic point cloud service name
    public: std::string pcSrv = {"/science_data_srv"};

    /// \brief Point cloud message containing location of data
    public: ignition::msgs::PointCloudPacked pcMsg;

    /// \brief Temperature data to visualize
    public: ignition::msgs::Float_V tempMsg;

    /// \brief Chlorophyll data to visualize
    public: ignition::msgs::Float_V chlorMsg;

    /// \brief Salinity data to visualize
    public: ignition::msgs::Float_V salMsg;

    /// \brief Performance trick. Cap number of points to visualize, to save
    /// memory.
    public: const int MAX_PTS_VIS = 1000;

    /// \brief Performance trick. Render only every other n. Increase to render
    /// fewer markers (faster performance).
    public: int renderEvery = 20;

    /// \brief Performance trick. Skip depths below this z, so have memory to
    /// visualize higher layers at higher resolution.
    /// For less confusion, match the parameter in ScienceSensorsSystem.cc.
    public: const float SKIP_Z_BELOW = -40;

    /// \brief Scale down to see in view to skip orbit tool limits
    /// For less confusion, match the parameter in ScienceSensorsSystem.cc.
    // TODO This is a workaround pending upstream orbit tool improvements
    // For 2003080103_mb_l3_las_1x1km.csv
    //public: const float MINIATURE_SCALE = 0.01;
    // For 2003080103_mb_l3_las.csv
    public: const float MINIATURE_SCALE = 0.0001;

    /// \brief Performance trick. Factor to multiply in calculating Marker sizes
    public: float dimFactor = 0.03;

    /// \brief Parameter to calculate Marker size in x.
    /// Performance trick. Hardcode resolution to make markers resemble voxels.
    // For 2003080103_mb_l3_las_1x1km.csv
    //public: const float RES_X = 15 * MINIATURE_SCALE;
    //public: const float RES_Y = 22 * MINIATURE_SCALE;
    //public: const float RES_Z = 5 * MINIATURE_SCALE;
    // For 2003080103_mb_l3_las.csv
    public: const float RES_X = 15;

    /// \brief Parameter to calculate Marker size in y.
    public: const float RES_Y = 22;

    /// \brief Parameter to calculate Marker size in z.
    public: const float RES_Z = 10;
  };
}

using namespace tethys;

/////////////////////////////////////////////////
VisualizePointCloud::VisualizePointCloud()
  : ignition::gui::Plugin(),
    dataPtr(std::make_unique<VisualizePointCloudPrivate>())
{
}

/////////////////////////////////////////////////
VisualizePointCloud::~VisualizePointCloud()
{
  this->ClearMarkers();
}

/////////////////////////////////////////////////
void VisualizePointCloud::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualize point cloud";

  if (!this->dataPtr->node.Subscribe("/temperature",
                            &VisualizePointCloud::OnTemperature, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << "/temperature" << "]\n";
    return;
  }
  if (!this->dataPtr->node.Subscribe("/chlorophyll",
                            &VisualizePointCloud::OnChlorophyll, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << "/chlorophyll" << "]\n";
    return;
  }
  if (!this->dataPtr->node.Subscribe("/salinity",
                            &VisualizePointCloud::OnSalinity, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << "/salinity" << "]\n";
    return;
  }

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void VisualizePointCloud::OnTopic(const QString &_topicName)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  // Unsubscribe from previous choice
  /*
  if (!this->dataPtr->topicName.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->topicName))
  {
    ignerr << "Unable to unsubscribe from topic ["
           << this->dataPtr->topicName <<"]" <<std::endl;
  }
  */
  this->dataPtr->topicName = _topicName.toStdString();

  // Request service
  this->dataPtr->node.Request(this->dataPtr->pcSrv,
      &VisualizePointCloud::OnService, this);

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->pcTopic,
                            &VisualizePointCloud::OnCloud, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << this->dataPtr->pcTopic << "]\n";
    return;
  }
  ignmsg << "Subscribed to " << this->dataPtr->pcTopic << std::endl;

  // This doesn't work correctly. Values do not correspond to the right type
  // of data. Maybe doesn't have time to subscribe befores markers go out.
  // Better to subscribe individually - worked more reliably.
  /*
  if (!this->dataPtr->node.Subscribe(this->dataPtr->topicName,
                            &VisualizePointCloud::OnFloatData, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << this->dataPtr->topicName << "]\n";
    return;
  }
  ignmsg << "Subscribed to " << this->dataPtr->topicName << std::endl;
  */

  // Clear visualization
  this->ClearMarkers();
}

//////////////////////////////////////////////////
void VisualizePointCloud::Show(bool _show)
{
  if (_show)
  {
    this->PublishMarkers();
  }
  else
  {
    this->ClearMarkers();
  }
}

/////////////////////////////////////////////////
void VisualizePointCloud::OnRefresh()
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  ignmsg << "Refreshing topic list for point cloud messages." << std::endl;

  // Clear
  this->dataPtr->topicList.clear();

  bool gotCloud = false;

  // Get updated list
  std::vector<std::string> allTopics;
  this->dataPtr->node.TopicList(allTopics);
  for (auto topic : allTopics)
  {
    std::vector<ignition::transport::MessagePublisher> publishers;
    this->dataPtr->node.TopicInfo(topic, publishers);
    for (auto pub : publishers)
    {
      // Have a fixed topic for point cloud locations. Let user choose
      // which science data type to visualize
      if (pub.MsgTypeName() == "ignition.msgs.PointCloudPacked")
      {
        //this->dataPtr->topicList.push_back(QString::fromStdString(topic));
        //break;
        gotCloud = true;
      }
      else if (pub.MsgTypeName() == "ignition.msgs.Float_V")
      {
        this->dataPtr->topicList.push_back(QString::fromStdString(topic));
      }
    }
  }
  if (gotCloud && this->dataPtr->topicList.size() > 0)
  {
    this->OnTopic(this->dataPtr->topicList.at(0));
  }

  this->TopicListChanged();
}

/////////////////////////////////////////////////
QStringList VisualizePointCloud::TopicList() const
{
  return this->dataPtr->topicList;
}

/////////////////////////////////////////////////
void VisualizePointCloud::SetTopicList(const QStringList &_topicList)
{
  this->dataPtr->topicList = _topicList;
  this->TopicListChanged();
}

//////////////////////////////////////////////////
void VisualizePointCloud::OnCloud(const ignition::msgs::PointCloudPacked &_msg)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->pcMsg = _msg;
  this->PublishMarkers();
}

/*
//////////////////////////////////////////////////
void VisualizePointCloud::OnFloatData(const ignition::msgs::Float_V &_msg)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->valMsg = _msg;
}
*/

//////////////////////////////////////////////////
void VisualizePointCloud::OnTemperature(const ignition::msgs::Float_V &_msg)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->tempMsg = _msg;
}

//////////////////////////////////////////////////
void VisualizePointCloud::OnChlorophyll(const ignition::msgs::Float_V &_msg)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->chlorMsg = _msg;
}

//////////////////////////////////////////////////
void VisualizePointCloud::OnSalinity(const ignition::msgs::Float_V &_msg)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->salMsg = _msg;
}

//////////////////////////////////////////////////
void VisualizePointCloud::OnService(const ignition::msgs::PointCloudPacked &_msg,
    bool _result)
{
  if (!_result)
  {
    ignerr << "Service request failed." << std::endl;
    return;
  }

  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->pcMsg = _msg;
  this->PublishMarkers();
}

//////////////////////////////////////////////////
void VisualizePointCloud::PublishMarkers()
{
  // If point cloud empty, do nothing. (PointCloudPackedIteratorBase errors on
  // empty cloud.)
  if (this->dataPtr->pcMsg.height() == 0 && this->dataPtr->pcMsg.width() == 0)
  {
    return;
  }

  // Used to calculate cap of number of points to visualize, to save memory
  int nPts = this->dataPtr->pcMsg.height() * this->dataPtr->pcMsg.width();
  this->dataPtr->renderEvery = (int) round(
    nPts / (double) this->dataPtr->MAX_PTS_VIS);

  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  ignition::msgs::Marker_V markers;

  PointCloudPackedIterator<float> iterX(this->dataPtr->pcMsg, "x");
  PointCloudPackedIterator<float> iterY(this->dataPtr->pcMsg, "y");
  PointCloudPackedIterator<float> iterZ(this->dataPtr->pcMsg, "z");
  // FIXME: publish point cloud fields instead of float arrays
  //PointCloudPackedIterator<float> iterTemp(this->dataPtr->pcMsg, "temperature");

  // Type of data to visualize
  std::string dataType = this->dataPtr->topicName;
  // Ranges to scale marker colors
  float minVal = 0.0f;
  float maxVal = 10000.0f;
  if (dataType == "/temperature")
  {
    minVal = 6.0f;
    maxVal = 20.0f;
  }
  else if (dataType == "/chlorophyll")
  {
    //minVal = -6.0f;
    minVal = 0.0f;
    maxVal = 6.5f;
  }
  else if (dataType == "/salinity")
  {
    minVal = 32.0f;
    maxVal = 34.5f;
  }

  igndbg << "First point in cloud (size "
         << this->dataPtr->pcMsg.height() * this->dataPtr->pcMsg.width()
         << "): " << *iterX << ", " << *iterY << ", " << *iterZ << std::endl;

  // Index of point in point cloud, visualized or not
  int ptIdx{0};
  // Number of points actually visualized
  int nPtsViz{0};
  while (iterX != iterX.end() &&
         iterY != iterY.end() &&
         iterZ != iterZ.end())
  {
    // Performance trick. Only publish every nth. Skip z below some depth
    if (this->dataPtr->renderEvery != 0 &&
        ptIdx % this->dataPtr->renderEvery == 0 &&
        *iterZ > this->dataPtr->SKIP_Z_BELOW)
    {
      // Science data value
      float dataVal = std::numeric_limits<float>::quiet_NaN();
      /*
      if (this->dataPtr->valMsg.data().size() > ptIdx)
      {
        dataVal = this->dataPtr->valMsg.data(ptIdx);
      }
      */
      // Sanity check array size
      if (dataType == "/temperature")
      {
        if (this->dataPtr->tempMsg.data().size() > ptIdx)
        {
          dataVal = this->dataPtr->tempMsg.data(ptIdx);
        }
      }
      else if (dataType == "/chlorophyll")
      {
        if (this->dataPtr->chlorMsg.data().size() > ptIdx)
        {
          dataVal = this->dataPtr->chlorMsg.data(ptIdx);
        }
      }
      else if (dataType == "/salinity")
      {
        if (this->dataPtr->salMsg.data().size() > ptIdx)
        {
          dataVal = this->dataPtr->salMsg.data(ptIdx);
        }
      }

      // Don't visualize NaN
      if (!std::isnan(dataVal))
      {
        auto msg = markers.add_marker();

        msg->set_ns(this->dataPtr->pcTopic);
        msg->set_id(nPtsViz + 1);

        msg->mutable_material()->mutable_ambient()->set_r(
          (dataVal - minVal) / (maxVal - minVal));
        msg->mutable_material()->mutable_ambient()->set_g(
          1 - (dataVal - minVal) / (maxVal - minVal));
        msg->mutable_material()->mutable_ambient()->set_a(0.5);

        msg->mutable_material()->mutable_diffuse()->set_r(
          (dataVal - minVal) / (maxVal - minVal));
        msg->mutable_material()->mutable_diffuse()->set_g(
          1 - (dataVal - minVal) / (maxVal - minVal));
        msg->mutable_material()->mutable_diffuse()->set_a(0.5);
        msg->set_action(ignition::msgs::Marker::ADD_MODIFY);

        // TODO: Use POINTS or LINE_LIST, but need per-vertex color
        msg->set_type(ignition::msgs::Marker::BOX);
        msg->set_visibility(ignition::msgs::Marker::GUI);
        //ignition::msgs::Set(msg->mutable_scale(),
        //                ignition::math::Vector3d::One);
        // Performance trick. Make boxes exact dimension of x and y gaps to
        // resemble "voxels". Then scale up by renderEvery to cover the space
        // where all the points are skipped.
        float dimX = this->dataPtr->RES_X * this->dataPtr->MINIATURE_SCALE
          * this->dataPtr->renderEvery * this->dataPtr->renderEvery
          * this->dataPtr->dimFactor;
        float dimY = this->dataPtr->RES_Y * this->dataPtr->MINIATURE_SCALE
          * this->dataPtr->renderEvery * this->dataPtr->renderEvery
          * this->dataPtr->dimFactor;
        float dimZ = this->dataPtr->RES_Z * this->dataPtr->MINIATURE_SCALE
          * this->dataPtr->renderEvery * this->dataPtr->renderEvery
          * this->dataPtr->dimFactor;
        ignition::msgs::Set(msg->mutable_scale(),
          ignition::math::Vector3d(dimX, dimY, dimZ));

        ignition::msgs::Set(msg->mutable_pose(), ignition::math::Pose3d(
          *iterX,
          *iterY,
          *iterZ,
          0, 0, 0));

        /*
        // Use POINTS type and array for better performance, pending per-point
        // color.
        // One marker per point cloud, many points.
        // TODO Implement in ign-gazebo per-point color like RViz point arrays,
        // so can have just 1 marker, many points in it, each with a specified
        // color, to improve performance. Color is the limiting factor that
        // requires us to use many markers here, 1 point per marker.
        // https://github.com/osrf/lrauv/issues/52
        ignition::msgs::Set(msg->mutable_pose(), ignition::math::Pose3d(
          0, 0, 0, 0, 0, 0));
        auto pt = msg->add_point();
        pt->set_x(*iterX);
        pt->set_y(*iterY);
        pt->set_z(*iterZ);
        */

        if (nPtsViz < 10)
        {
          igndbg << "Added point " << nPtsViz << " at "
                 << msg->pose().position().x() << ", "
                 << msg->pose().position().y() << ", "
                 << msg->pose().position().z() << ", "
                 << "value " << dataVal << ", "
                 << "type " << dataType << ", "
                 << "dimX " << dimX
                 << std::endl;
        }
        ++nPtsViz;
      }
    }

    ++iterX;
    ++iterY;
    ++iterZ;
    ++ptIdx;
  }

  ignerr << "Visualizing " << markers.marker().size() << " markers"
    << std::endl;

  ignition::msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  this->dataPtr->node.Request("/marker_array", markers, timeout, res, result);

  if (!result || !res.data())
  {
    ignerr << "Failed to create markers on /marker_array" << std::endl;
  }
}

//////////////////////////////////////////////////
void VisualizePointCloud::ClearMarkers()
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  ignition::msgs::Marker msg;
  msg.set_ns(this->dataPtr->pcTopic);
  msg.set_id(0);
  msg.set_action(ignition::msgs::Marker::DELETE_ALL);

  this->dataPtr->node.Request("/marker", msg);
}

// Register this plugin
IGNITION_ADD_PLUGIN(tethys::VisualizePointCloud,
                    ignition::gui::Plugin)
