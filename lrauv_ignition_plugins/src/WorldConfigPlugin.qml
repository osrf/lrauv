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
import QtQuick.Dialogs 1.0

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
    text: "Load Environment Data CSV"
    Layout.columnSpan: 3
  }
  Button {
    id: vehicleNameValue
    Layout.columnSpan: 2
    Layout.fillWidth: true
    text: "Browse files..."
    onClicked: fileDialog.open()
  }

  FileDialog {
    id: fileDialog
    title: "Please choose a file"
    folder: shortcuts.home
    visible: false
    onAccepted: {
      WorldConfig.SetFilePath(fileDialog.fileUrl)
    }
    onRejected: {
    }
    //Component.onCompleted: visible = true
  }
}
