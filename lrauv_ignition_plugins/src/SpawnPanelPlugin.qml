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
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import ignition.gui 1.0

GridLayout {
  id: mainLayout
  columns: 5
  rowSpacing: 2
  columnSpacing: 2
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10
  Layout.minimumWidth: 400
  Layout.minimumHeight: 300

  Label {
    text: "Vehicle name"
    Layout.columnSpan: 1
  }
  TextField {
    id: vehicleNameValue
    Layout.columnSpan: 4
    Layout.fillWidth: true
    text: "tethys"
  }

  Label {
    text: "Latitude"
    Layout.columnSpan: 1
  }
  TextField {
    id: latitudeValue
    Layout.columnSpan: 4
    Layout.fillWidth: true
    text: "120.000"
  }

  Label {
    text: "Longitude"
    Layout.columnSpan: 1
  }
  TextField {
    id: longitudeValue
    Layout.columnSpan: 4
    Layout.fillWidth: true
    text: "30.000"
  }

  Label {
    text: "Depth"
    Layout.columnSpan: 1
  }
  TextField {
    id: depthValue
    Layout.columnSpan: 4
    Layout.fillWidth: true
    text: "0"
  }

  Label {
    text: "Comms ID"
    Layout.columnSpan: 1
  }
  IgnSpinBox {
    id: commsValue
    Layout.columnSpan: 4
    value: 0
    minimumValue: 0
    maximumValue: 255
    decimals: 0
    stepSize: 1
  }

  Button {
    text: "Spawn!"
    Layout.columnSpan: 5
    onClicked: function() {
      SpawnPanel.Spawn(
        parseFloat(latitudeValue.text),
        parseFloat(longitudeValue.text),
        parseFloat(depthValue.text),
        parseInt(commsValue.value),
        vehicleNameValue.text
      )
    }
  }
}
