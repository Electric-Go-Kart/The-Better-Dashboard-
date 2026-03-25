

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
import QtQuick.Shapes
import QMLCDash

Item {
    id: root
    anchors.fill: parent
    visible: true

    Rectangle {
        id: border
        anchors.fill: parent
        color: "#333333"


        Speedometer {
            id: speedometer
            anchors.horizontalCenter: parent.horizontalCenter
            y: parent.height * 0.3

        }

        DirectionButton {
            id: directionButton
            x: parent.width * 0.75
            y: parent.height * 0.55
        }



        CurrentDrawLeft {
            id: currentDrawLeft
            x: parent.width * 0.925 - parent.width * 0.25
            y: parent.height * 0.075

            BatteryGaugeLeft {
                id: batteryGaugeLeft
                anchors.horizontalCenter: parent.horizontalCenter
                y: parent.height * 0.55
            }
        }

        CurrentDrawRight {
            id: currentDrawRight
            x: parent.width * 0.075
            y: parent.height * 0.075

            BatteryGaugeRight {
                id: batteryGaugeRight
                anchors.horizontalCenter: parent.horizontalCenter
                y: parent.height * 0.55
            }
        }

        Park {
            id: park
            x: parent.width * 0.75
            y: parent.height * 0.7
        }

        Lights {
            id: lights
            x: parent.width * 0.75
            y: parent.height * 0.85
        }

        ShutDown {
            id: shutDown
            x: parent.width * 0.125
            y: parent.height * 0.55
        }

        Settings {
            id: settings
            x: parent.width * 0.125
            y: parent.height * 0.7
        }

        Lock {
            id: lock
            x: parent.width * 0.125
            y: parent.height * 0.85
        }
    }


}
