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

ColumnLayout {
  spacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 350
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  function isUniform() {
    return VisualizePointCloud.minFloatV >= VisualizePointCloud.maxFloatV
  }

  RowLayout {
    spacing: 10
    Layout.fillWidth: true

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
  }

  GridLayout {
    columns: 3
    columnSpacing: 10
    Layout.fillWidth: true

    Label {
      Layout.columnSpan: 1
      text: "Point cloud"
    }

    ComboBox {
      Layout.columnSpan: 2
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
      Layout.columnSpan: 2
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
      ToolTip.text: qsTr("Ignition transport topics publishing FloatV messages, used to color each point on the cloud")
    }

    Label {
      Layout.columnSpan: 1
      text: "Point size"
    }

    IgnSpinBox {
      id: pointSizeSpin
      value: VisualizePointCloud.pointSize
      minimumValue: 1
      maximumValue: 1000
      decimals: 0
      onEditingFinished: {
        VisualizePointCloud.SetPointSize(pointSizeSpin.value)
      }
    }
  }

  RowLayout {
    spacing: 10
    Layout.fillWidth: true

    Label {
      Layout.columnSpan: 1
      text: isUniform() ? "Color" : "Min"
    }

    Label {
      Layout.columnSpan: 1
      Layout.maximumWidth: 50
      text: VisualizePointCloud.minFloatV.toFixed(4)
      elide: Text.ElideRight
      visible: !isUniform()
    }

    Button {
      Layout.columnSpan: 1
      id: minColorButton
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Color for minimum value")
      onClicked: minColorDialog.open()
      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        radius: 5
        border.color: VisualizePointCloud.minColor
        border.width: 2
        color: VisualizePointCloud.minColor
      }
      ColorDialog {
        id: minColorDialog
        title: "Choose a color for the minimum value"
        visible: false
        onAccepted: {
          VisualizePointCloud.SetMinColor(minColorDialog.color)
          minColorDialog.close()
        }
        onRejected: {
          minColorDialog.close()
        }
      }
    }

    Button {
      Layout.columnSpan: 1
      id: maxColorButton
      visible: !isUniform()
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Color for maximum value")
      onClicked: maxColorDialog.open()
      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        radius: 5
        border.color: VisualizePointCloud.maxColor
        border.width: 2
        color: VisualizePointCloud.maxColor
      }
      ColorDialog {
        id: maxColorDialog
        title: "Choose a color for the maximum value"
        visible: false
        onAccepted: {
          VisualizePointCloud.SetMaxColor(maxColorDialog.color)
          maxColorDialog.close()
        }
        onRejected: {
          maxColorDialog.close()
        }
      }
    }

    Label {
      Layout.columnSpan: 1
      Layout.maximumWidth: 50
      text: VisualizePointCloud.maxFloatV.toFixed(4)
      elide: Text.ElideRight
      visible: !isUniform()
    }

    Label {
      Layout.columnSpan: 1
      text: "Max"
      visible: !isUniform()
    }
  }

  Item {
    Layout.columnSpan: 6
    width: 10
    Layout.fillHeight: true
  }
}
