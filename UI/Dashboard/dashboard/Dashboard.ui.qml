

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
import QMLCDash

Item {
    id: root
    width: 800
    height: 480
    signal settingsRequested()
    property string footerWarning: ""
    property bool footerWarningVisible: false

    Rectangle {
        anchors.fill: parent
        color: "#0f1c18"
    }

    Rectangle {
        x: 16
        y: 12
        width: 768
        height: 44
        radius: 8
        color: "#17362f"
        border.width: 1
        border.color: "#c8b273"

        Text {
            anchors.left: parent.left
            anchors.leftMargin: 12
            anchors.verticalCenter: parent.verticalCenter
            text: "CSU Drive " + dashboardController.driveState + " | " + dashboardController.driveMode
            color: "#f5f5f5"
            font.pixelSize: 17
            font.bold: true
        }

        Text {
            anchors.right: parent.right
            anchors.rightMargin: 12
            anchors.verticalCenter: parent.verticalCenter
            text: dashboardController.canOnline ? "CAN Online" : "CAN Offline"
            color: dashboardController.canOnline ? "#87d88d" : "#ff7b7b"
            font.pixelSize: 16
            font.bold: true
        }
    }

    Speedometer {
        id: speedometer
        x: 225
        y: 78
    }

    DirectionButton {
        id: directionButton
        x: 606
        y: 84
    }

    BatteryGauge {
        id: batteryGauge
        x: 743
        y: 60
    }

    CurrentDraw {
        id: currentDraw
        x: 37
        y: 60
    }

    Park {
        id: park
        x: 606
        y: 149
    }

    Lights {
        id: lights
        x: 606
        y: 214
    }

    ShutDown {
        id: shutDown
        x: 94
        y: 84
    }

    Settings {
        id: settings
        x: 94
        y: 149
        onSettingsPressed: root.settingsRequested()
    }

    Lock {
        id: lock
        x: 94
        y: 214
    }

    Rectangle {
        x: 16
        y: 440
        width: 768
        height: 24
        radius: 5
        visible: root.footerWarning.length > 0 || opacity > 0
        opacity: root.footerWarningVisible ? 1 : 0
        color: "#17362f"
        border.width: 1
        border.color: "#c8b273"

        Behavior on opacity {
            NumberAnimation {
                duration: 320
                easing.type: Easing.InOutQuad
            }
        }

        Text {
            anchors.centerIn: parent
            text: root.footerWarning
            color: "#ffd2d2"
            font.pixelSize: 14
        }
    }

    Connections {
        target: dashboardController
        onLastControlErrorChanged: function(errorMessage) {
            if (errorMessage.length > 0) {
                clearWarningTimer.stop()
                root.footerWarning = errorMessage
                root.footerWarningVisible = true
                warningResetTimer.restart()
            }
        }
    }

    Timer {
        id: warningResetTimer
        interval: 3000
        repeat: false
        onTriggered: {
            root.footerWarningVisible = false
            clearWarningTimer.restart()
        }
    }

    Timer {
        id: clearWarningTimer
        interval: 360
        repeat: false
        onTriggered: root.footerWarning = ""
    }
}
