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

    /// \brief Topic name to subscribe
    public: std::string topicName{""};

    /// \brief List of topics publishing LaserScan messages.
    public: QStringList topicList;

    /// \brief Protect variables changed from transport and the user
    public: std::recursive_mutex mutex;

    /// \brief Latest point cloud message
    public: ignition::msgs::PointCloudPacked pcMsg;
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

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void VisualizePointCloud::OnTopic(const QString &_topicName)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  if (!this->dataPtr->topicName.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->topicName))
  {
    ignerr << "Unable to unsubscribe from topic ["
           << this->dataPtr->topicName <<"]" <<std::endl;
  }

  // Clear visualization
  this->ClearMarkers();

  this->dataPtr->topicName = _topicName.toStdString();

  // Request service
  this->dataPtr->node.Request(this->dataPtr->topicName,
      &VisualizePointCloud::OnService, this);

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->topicName,
                            &VisualizePointCloud::OnMessage, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << this->dataPtr->topicName << "]\n";
    return;
  }
  ignmsg << "Subscribed to " << this->dataPtr->topicName << std::endl;
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

  // Get updated list
  std::vector<std::string> allTopics;
  this->dataPtr->node.TopicList(allTopics);
  for (auto topic : allTopics)
  {
    std::vector<ignition::transport::MessagePublisher> publishers;
    this->dataPtr->node.TopicInfo(topic, publishers);
    for (auto pub : publishers)
    {
      if (pub.MsgTypeName() == "ignition.msgs.PointCloudPacked")
      {
        this->dataPtr->topicList.push_back(QString::fromStdString(topic));
        break;
      }
    }
  }
  if (this->dataPtr->topicList.size() > 0)
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
void VisualizePointCloud::OnMessage(const ignition::msgs::PointCloudPacked &_msg)
{
  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->pcMsg = _msg;
  this->PublishMarkers();
}

//////////////////////////////////////////////////
void VisualizePointCloud::OnService(const ignition::msgs::PointCloudPacked &_res,
    bool _result)
{
  if (!_result)
  {
    ignerr << "Service request failed." << std::endl;
    return;
  }

  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  this->dataPtr->pcMsg = _res;
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

  std::lock_guard<std::recursive_mutex>(this->dataPtr->mutex);
  ignition::msgs::Marker_V markers;

  PointCloudPackedIterator<float> iter_x(this->dataPtr->pcMsg, "x");
  PointCloudPackedIterator<float> iter_y(this->dataPtr->pcMsg, "y");
  PointCloudPackedIterator<float> iter_z(this->dataPtr->pcMsg, "z");

  int count{0};
  while (iter_x != iter_x.end() &&
         iter_y != iter_y.end() &&
         iter_z != iter_z.end())
  {
    auto msg = markers.add_marker();

    msg->set_ns(this->dataPtr->topicName);
    msg->set_id(count++);
    msg->mutable_material()->mutable_ambient()->set_b(1);
    msg->mutable_material()->mutable_ambient()->set_a(0.3);
    msg->mutable_material()->mutable_diffuse()->set_b(1);
    msg->mutable_material()->mutable_diffuse()->set_a(0.3);
    msg->set_action(ignition::msgs::Marker::ADD_MODIFY);
    msg->set_type(ignition::msgs::Marker::BOX);
    msg->set_visibility(ignition::msgs::Marker::GUI);
    ignition::msgs::Set(msg->mutable_scale(),
                    ignition::math::Vector3d::One);

    ignition::msgs::Set(msg->mutable_pose(), ignition::math::Pose3d(
      *iter_x,
      *iter_y,
      *iter_z,
      0, 0, 0));

    ++iter_x;
    ++iter_y;
    ++iter_z;

    // FIXME: how to handle more points?
    // https://github.com/osrf/lrauv/issues/85
    if (count > 100)
      break;
  }

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
  msg.set_ns(this->dataPtr->topicName);
  msg.set_id(0);
  msg.set_action(ignition::msgs::Marker::DELETE_ALL);

  this->dataPtr->node.Request("/marker", msg);
}

// Register this plugin
IGNITION_ADD_PLUGIN(tethys::VisualizePointCloud,
                    ignition::gui::Plugin)
