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

#ifndef TETHYS_VISUALIZEPOINTCLOUD_HH_
#define TETHYS_VISUALIZEPOINTCLOUD_HH_

#include <memory>

#include "ignition/msgs/float_v.pb.h"
#include "ignition/msgs/pointcloud_packed.pb.h"

#include <ignition/gui/Plugin.hh>

namespace tethys
{
  class VisualizePointCloudPrivate;

  /// \brief Visualize PointCloudPacked messages in the 3D scene.
  class VisualizePointCloud : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief List of topics publishing PointCloudPacked messages
    Q_PROPERTY(
      QStringList topicList
      READ TopicList
      WRITE SetTopicList
      NOTIFY TopicListChanged
    )

    /// \brief Constructor
    public: VisualizePointCloud();

    /// \brief Destructor
    public: ~VisualizePointCloud() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback function to get data from the message
    /// \param[in]_msg Point cloud message
    public: void OnCloud(const ignition::msgs::PointCloudPacked &_msg);

    // TODO TEMPORARY HACK float arrays instead of point cloud fields
    public: void OnTemperature(const ignition::msgs::Float_V &_msg);
    public: void OnChlorophyll(const ignition::msgs::Float_V &_msg);
    public: void OnSalinity(const ignition::msgs::Float_V &_msg);
    public: void OnFloatData(const ignition::msgs::Float_V &_msg);

    /// \brief Callback function to get data from the message
    /// \param[in]_msg Point cloud message
    /// \param[out]_result True on success.
    public: void OnService(const ignition::msgs::PointCloudPacked &_msg,
        bool _result);

    /// \brief Get the topic list
    /// \return List of topics
    public: Q_INVOKABLE QStringList TopicList() const;

    /// \brief Set the topic list from a string
    /// \param[in] _topicList List of topics.
    public: Q_INVOKABLE void SetTopicList(const QStringList &_topicList);

    /// \brief Notify that topic list has changed
    signals: void TopicListChanged();

    /// \brief Set topic to subscribe to.
    /// \param[in] _topicName Name of selected topic
    public: Q_INVOKABLE void OnTopic(const QString &_topicName);

    /// \brief Set whether to show the point cloud.
    /// \param[in] _show Boolean value for displaying the points.
    public: Q_INVOKABLE void Show(bool _show);

    /// \brief Callback when refresh button is pressed.
    public: Q_INVOKABLE void OnRefresh();

    /// \brief Makes a request to populate the scene with markers
    private: void PublishMarkers();

    /// \brief Makes a request to delete all markers related to the point cloud.
    private: void ClearMarkers();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<VisualizePointCloudPrivate> dataPtr;
  };
}
#endif
