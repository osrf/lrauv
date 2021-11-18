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

import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 200
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  Switch {
    Layout.alignment: Qt.AlignHCenter
    id: displayVisual
    Layout.columnSpan: 5
    Layout.fillWidth: true
    text: qsTr("Show")
    checked: true
    onToggled: {
      VisualizePointCloud.Show(checked)
    }
  }

  RoundButton {
    Layout.columnSpan: 1
    text: "\u21bb"
    Material.background: Material.primary
    onClicked: {
      VisualizePointCloud.OnRefresh();
      pcCombo.currentIndex = 0
      floatCombo.currentIndex = 0
    }
    ToolTip.visible: hovered
    ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
    ToolTip.text: qsTr("Refresh list of topics publishing point clouds and float vectors")
  }

  Label {
    Layout.columnSpan: 1
    text: "Point cloud"
  }

  ComboBox {
    Layout.columnSpan: 5
    id: pcCombo
    Layout.fillWidth: true
    model: VisualizePointCloud.pointCloudTopicList
    currentIndex: 0
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      VisualizePointCloud.OnPointCloudTopic(textAt(currentIndex));
    }
    ToolTip.visible: hovered
    ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
    ToolTip.text: qsTr("Ignition transport topics publishing PointCloudPacked messages")
  }

  Label {
    Layout.columnSpan: 1
    text: "Float vector"
  }

  ComboBox {
    Layout.columnSpan: 5
    id: floatCombo
    Layout.fillWidth: true
    model: VisualizePointCloud.floatVTopicList
    currentIndex: 0
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      VisualizePointCloud.OnFloatVTopic(textAt(currentIndex));
    }
    ToolTip.visible: hovered
    ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
    ToolTip.text: qsTr("Ignition transport topics publishing FloatV messages")
  }

  Item {
    Layout.columnSpan: 6
    width: 10
    Layout.fillHeight: true
  }
}
