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
    onEditingFinished: function () {
      ControlPanel.SetVehicle(text);
    }
  }

  // Rudder
  property double rudderValue: 0
  property double rudderMin: -0.26
  property double rudderMax: 0.26
  Label {
    text: "Rudder (rad)"
    Layout.columnSpan: 1
  }
  IgnSpinBox {
    id: rudderSpin
    Layout.columnSpan: 1
    value: rudderValue
    minimumValue: rudderMin
    maximumValue: rudderMax
    decimals: 2
    stepSize: 0.01
    onEditingFinished: function () {
      rudderValue = value;
      ControlPanel.SetRudder(value);
    }
  }
  Label {
    text: rudderMin
    Layout.columnSpan: 1
    font.pixelSize: 9
    Layout.alignment: Qt.AlignRight
  }
  Slider {
    id: rudderControl
    Layout.columnSpan: 1
    Layout.fillWidth: true
    from: rudderMin
    to: rudderMax
    value: rudderValue
    onMoved: function() {
      rudderValue = Math.round(value * 100) / 100;
      ControlPanel.SetRudder(value);
    }
  }
  Label {
    text: rudderMax
    Layout.columnSpan: 1
    font.pixelSize: 9
  }

  // Elevator
  property double elevatorValue: 0
  property double elevatorMin: -0.26
  property double elevatorMax: 0.26
  Label {
    text: "Elevator (rad)"
    Layout.columnSpan: 1
  }
  IgnSpinBox {
    id: elevatorSpin
    Layout.columnSpan: 1
    value: elevatorValue
    minimumValue: elevatorMin
    maximumValue: elevatorMax
    decimals: 2
    stepSize: 0.01
    onEditingFinished: function () {
      elevatorControl.value = value;
      ControlPanel.SetElevator(value);
    }
  }
  Label {
    text: elevatorMin
    Layout.columnSpan: 1
    font.pixelSize: 9
    Layout.alignment: Qt.AlignRight
  }
  Slider {
    id: elevatorControl
    Layout.columnSpan: 1
    Layout.fillWidth: true
    from: elevatorMin
    to: elevatorMax
    value: elevatorValue
    onMoved: function() {
      elevatorValue = Math.round(value * 100) / 100;
      ControlPanel.SetElevator(elevatorValue);
    }
  }
  Label {
    text: elevatorMax
    Layout.columnSpan: 1
    font.pixelSize: 9
  }

  // Mass
  property double massValue: 0
  property double massMin: -0.03
  property double massMax: 0.03
  Label {
    text: "Mass shifter (m)"
    Layout.columnSpan: 1
  }
  IgnSpinBox {
    id: massSpin
    Layout.columnSpan: 1
    value: massValue
    minimumValue: massMin
    maximumValue: massMax
    decimals: 3
    stepSize: 0.001
    onEditingFinished: function () {
      massValue = value;
      ControlPanel.SetMass(value);
    }
  }
  Label {
    text: massMin
    Layout.columnSpan: 1
    font.pixelSize: 9
    Layout.alignment: Qt.AlignRight
  }
  Slider {
    id: massControl
    Layout.columnSpan: 1
    Layout.fillWidth: true
    from: massMin
    to: massMax
    value: massValue
    onMoved: function() {
      massValue = Math.round(value * 100) / 100;
      ControlPanel.SetPitchMass(value);
    }
  }
  Label {
    text: massMax
    Layout.columnSpan: 1
    font.pixelSize: 9
  }

  // Thruster Control
  property double thrusterValue: 0
  property double thrusterMin: -30
  property double thrusterMax: 30
  Label {
    text: "Thruster (rad/s)"
    Layout.columnSpan: 1
  }
  IgnSpinBox {
    id: thrusterSpin
    Layout.columnSpan: 1
    value: thrusterValue
    minimumValue: thrusterMin
    maximumValue: thrusterMax
    decimals: 1
    stepSize: 1.0
    onEditingFinished: function () {
      thrusterValue = value;
      ControlPanel.SetThruster(value);
    }
  }
  Label {
    text: thrusterMin
    Layout.columnSpan: 1
    font.pixelSize: 9
    Layout.alignment: Qt.AlignRight
  }
  Slider {
    id: thrustControl
    Layout.columnSpan: 1
    Layout.fillWidth: true
    from: thrusterMin
    to: thrusterMax
    value: thrusterValue
    onMoved: function() {
      thrusterValue = Math.round(value * 100) / 100;
      ControlPanel.SetThruster(value);
    }
  }
  Label {
    text: thrusterMax
    Layout.columnSpan: 1
    font.pixelSize: 9
  }

  // Buoyancy engine
  property double buoyancyValue: 500
  property double buoyancyMin: 0
  property double buoyancyMax: 900
  Label {
    text: "Buoyancy engine (cc)"
    Layout.columnSpan: 1
  }
  IgnSpinBox {
    id: buoyancySpin
    Layout.columnSpan: 1
    value: buoyancyValue
    minimumValue: buoyancyMin
    maximumValue: buoyancyMax
    decimals: 0
    stepSize: 0.01
    onEditingFinished: function () {
      buoyancyValue = value;
      var valueCubicMeters = value / (100 * 100 * 100);
      ControlPanel.SetBuoyancy(valueCubicMeters);
    }
  }
  Label {
    text: buoyancyMin
    Layout.columnSpan: 1
    font.pixelSize: 9
    Layout.alignment: Qt.AlignRight
  }
  Slider {
    id: buoyancyControl
    Layout.columnSpan: 1
    Layout.fillWidth: true
    from: buoyancyMin
    to: buoyancyMax
    value: buoyancyValue
    onMoved: function() {
      buoyancyValue = Math.round(value);
      var valueCubicMeters = value / (100 * 100 * 100);
      ControlPanel.SetBuoyancyEngine(valueCubicMeters);
    }
  }
  Label {
    text: buoyancyMax
    Layout.columnSpan: 1
    font.pixelSize: 9
  }

  // Drop weight
  Label {
    text: "Drop weight"
    Layout.columnSpan: 1
  }
  Label {
    text: "Detached"
    Layout.columnSpan: 1
    font.pixelSize: 9
    Layout.alignment: Qt.AlignRight
  }
  Switch {
    id: dropWeightRelease
    Layout.columnSpan: 1
    Layout.fillWidth: true
    checked: true
    onToggled: {
      ControlPanel.ReleaseDropWeight();
      checkable = false;
    }
  }
  Label {
    text: "Attached"
    Layout.columnSpan: 2
    font.pixelSize: 9
  }

  Item {
    Layout.columnSpan: 3
    width: 10
    Layout.fillHeight: true
  }
}
