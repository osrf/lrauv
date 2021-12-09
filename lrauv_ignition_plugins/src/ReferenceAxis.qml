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
  columns: 1
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 200
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  Label {
    Layout.columnSpan: 1
    text: "No configuration options at the moment."
  }


  Item {
    Layout.columnSpan: 1
    width: 10
    Layout.fillHeight: true
  }
}
