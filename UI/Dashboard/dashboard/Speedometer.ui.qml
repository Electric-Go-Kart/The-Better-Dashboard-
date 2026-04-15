

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
import Qt5Compat.GraphicalEffects
import QMLCDash
import "Theme.js" as Theme

Item {
    id: root
    width: 325
    height: 325
    signal logoPressed()
    property real speedMph: 0
    property bool demoMode: false
    property real demoSpeedMph: 0
    property real wheelGearTeeth: 30
    property real motorGearTeeth: 9
    property real wheelDiameterInches: 9.5
    property real maxMph: 25
    property real dialStartDeg: -210
    property real dialSweepDeg: 240
    property int majorTickCount: 6

    function clampedMph(v) {
        return Math.max(0, Math.min(root.maxMph, v))
    }

    function displaySpeed() {
        const speedBaseMph = demoMode ? demoSpeedMph : speedMph
        return dashboardController.unitsSystem === "Metric"
                ? speedBaseMph * 1.60934
                : speedBaseMph
    }

    function unitLabel() {
        return dashboardController.unitsSystem === "Metric" ? "kph" : "mph"
    }

    function tickValue(index) {
        const mphValue = (root.maxMph / (root.majorTickCount - 1)) * index
        if (dashboardController.unitsSystem === "Metric") {
            return Math.round(mphValue * 1.60934)
        }
        return Math.round(mphValue)
    }

    function tickAngle(index) {
        return root.dialStartDeg + (root.dialSweepDeg * index / (root.majorTickCount - 1))
    }

    function maxDisplayValue() {
        return dashboardController.unitsSystem === "Metric"
                ? Math.round(root.maxMph * 1.60934)
                : Math.round(root.maxMph)
    }

    function angleForDisplayValue(displayValue) {
        return root.dialStartDeg + (root.dialSweepDeg * displayValue / root.maxDisplayValue())
    }

       Connections {
           target: dashboardController
           onLeftRpmChanged: {
               const wheelRpm = rpm * (root.motorGearTeeth / root.wheelGearTeeth)
               const wheelCircumferenceInches = Math.PI * root.wheelDiameterInches
               root.speedMph = (wheelRpm * wheelCircumferenceInches * 60.0) / 63360.0
           }
       }
    Rectangle {
        id: speedometer
        width: 325
        height: 325
        radius: 162.5
        border.width: 2
        border.color: Theme.csuGold
        gradient: Gradient {
            GradientStop {
                position: 0
                color: "#17362f"
            }

            GradientStop {
                position: 1
                color: "#0c1412"
            }
            orientation: Gradient.Vertical
        }

        Image {
                id: ramsLogoSource
                visible: false
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
                source: "../images/CSU-Ram-Rev.png"
                mipmap: true
                width: 204
                height: 204
            }

        ColorOverlay {
            id: ramsLogo
            anchors.fill: ramsLogoSource
            source: ramsLogoSource
            color: "#B7C8BE"
            opacity: dashboardController.lockEnabled ? 0.62 : 0.4
        }

        MouseArea {
            anchors.centerIn: ramsLogo
            width: 184
            height: 184
            onClicked: root.logoPressed()
        }

        Repeater {
            model: root.maxDisplayValue() + 1
            Rectangle {
                width: 2
                height: 8
                radius: 1
                color: "#C7D3CC"
                readonly property real angleDeg: root.angleForDisplayValue(index)
                readonly property real angleRad: angleDeg * Math.PI / 180.0
                readonly property real radiusPx: 157.5
                x: speedometer.width / 2 + Math.cos(angleRad) * radiusPx - width / 2
                y: speedometer.height / 2 + Math.sin(angleRad) * radiusPx - height / 2
                rotation: angleDeg + 90
                transformOrigin: Item.Center
            }
        }

        Repeater {
            model: root.majorTickCount
            Rectangle {
                width: 3
                height: 18
                radius: 1
                color: Theme.csuGold
                readonly property real angleDeg: root.tickAngle(index)
                readonly property real angleRad: angleDeg * Math.PI / 180.0
                readonly property real radiusPx: 153.5
                x: speedometer.width / 2 + Math.cos(angleRad) * radiusPx - width / 2
                y: speedometer.height / 2 + Math.sin(angleRad) * radiusPx - height / 2
                rotation: angleDeg + 90
                transformOrigin: Item.Center
            }
        }

        Repeater {
            model: root.majorTickCount
            Text {
                color: Theme.textMuted
                font.pixelSize: 14
                font.bold: true
                text: root.tickValue(index)
                readonly property real angleDeg: root.tickAngle(index)
                readonly property real angleRad: angleDeg * Math.PI / 180.0
                readonly property real radiusPx: 126
                x: speedometer.width / 2 + Math.cos(angleRad) * radiusPx - width / 2
                y: speedometer.height / 2 + Math.sin(angleRad) * radiusPx - height / 2
            }
        }

        Rectangle {
            id: needle
            width: 4
            height: 116
            radius: 2
            x: speedometer.width / 2 - width / 2
            y: speedometer.height / 2 - height
            antialiasing: true
            transformOrigin: Item.Bottom
            readonly property real effectiveMph: root.demoMode ? root.demoSpeedMph : root.speedMph
            rotation: root.dialStartDeg
                      + (root.clampedMph(effectiveMph) / root.maxMph) * root.dialSweepDeg
                      + 90
            Behavior on rotation {
                NumberAnimation {
                    duration: 180
                    easing.type: Easing.OutCubic
                }
            }
            gradient: Gradient {
                orientation: Gradient.Vertical
                GradientStop { position: 0.0; color: "#8A3E0E" }
                GradientStop { position: 0.55; color: Theme.warning }
                GradientStop { position: 1.0; color: "#D48E2F" }
            }
        }

        Text {
            id: speed
            x: 114
            y: 176
            width: 98
            color: Theme.textPrimary
            text: qsTr("%1").arg(root.displaySpeed().toFixed(1))
            font.pixelSize: 44
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
        }

        Text {
            id: text1
            x: 140
            y: 226
            color: Theme.textMuted
            text: qsTr(root.unitLabel())
            font.pixelSize: 22
            font.bold: true
        }
    }
}
