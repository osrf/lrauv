/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

GridLayout {
    id: mainLayout
    columns: 1
    rowSpacing: 5
    columnSpacing: 5
    anchors {
        top: parent.top;
        left: parent.left
        right: parent.right
    }

    Label { text: "Vehicle Name" }
    TextField {
      id: vehicleNameValue
      text: "tethys"
      onEditingFinished: function () {
        ControlPanel.SetVehicle(text);
      }
    }

    /// RUDDER Controls
    Label { text: "Rudder" }
    TextField {
      id: rudderValue
      text: "0"
      onEditingFinished: function () {
        rudderControl.value = parseFloat(text);
        ControlPanel.SetRudder(parseFloat(text));
      }
      validator: DoubleValidator {
        bottom: -0.26
        top: 0.26
      }
    }
    Slider {
      id: rudderControl
      from: -0.26
      value: 0
      to: 0.26
      onMoved: function() {
        rudderValue.text = Math.round(value * 100) / 100;
        ControlPanel.SetRudder(value);
      }
    }

    /// ELEV Controls
    Label { text: "Elevator"}
    TextField { id: elevatorValue
      text: "0"
      onEditingFinished: function () {
        elevatorControl.value = parseFloat(text);
        ControlPanel.SetElevator(parseFloat(text));
      }
      validator: DoubleValidator {
        bottom: -0.26
        top: 0.26
      }
    }
    Slider {
      id: elevatorControl
      orientation: Qt.Vertical
      from: -0.26
      to: 0.26
      onMoved: function() {
        elevatorValue.text = Math.round(value * 100) / 100;
        ControlPanel.SetElevator(value);
      }
    }

    /// Mass Controls
    Label { text: "Mass Shifter" }
    TextField { id: massValue
      text: "0"
      onEditingFinished: function () {
        massControl.value = parseFloat(text);
        ControlPanel.SetPitchMass(parseFloat(text));
      }
      validator: DoubleValidator {
        bottom: -0.03
        top: 0.03
      }
    }
    Slider {
      id: massControl
      from: -0.03
      to: 0.03
      onMoved: function() {
        massValue.text = Math.round(value * 100) / 100;
        ControlPanel.SetPitchMass(value);
      }
    }

    /// Thruster Control
    Label { text: "Thruster" }
    TextField { id: thrusterValue
      text: "0"
      onEditingFinished: function () {
        thrustControl.value = parseFloat(text);
        ControlPanel.SetThruster(parseFloat(text));
      }
      validator: DoubleValidator {
        bottom: -1
        top: 31.4152698
      }
    }
    Slider {
      id: thrustControl
      from: -1
      to: 31.4152698
      onMoved: function() {
        thrusterValue.text = Math.round(value * 100) / 100;
        ControlPanel.SetThruster(value);
      }
    }

    Label { text: "Buoyancy Engine" }
    TextField { id: buoyancyValue
      text: "0"
      onEditingFinished: function () {
        buoyancyControl.value = parseFloat(text);
        //ControlPanel.SetRudder(parseFloat(text));
      }
      validator: DoubleValidator {
        bottom: -0.26
        top: 0.26
      }
    }
    Slider {
      id: buoyancyControl
      from: 0
      to: 900
    }

    Button {
      id: dropWeightRelease
      text: "Release Drop Weight"
      onClicked: ControlPanel.ReleaseDropWeight()
    }
}
