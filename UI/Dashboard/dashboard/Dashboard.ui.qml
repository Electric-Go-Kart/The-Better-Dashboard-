

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
    signal vehicleExplorerRequested()
    property string footerWarning: ""
    property bool footerWarningVisible: false
    property bool demoSweepEnabled: typeof uiSweepTestEnabled !== "undefined" ? uiSweepTestEnabled : false
    property real demoSpeedMph: 0
    property real demoCurrentLeft: 0
    property real demoCurrentRight: 0
    property real demoSocLeft: 100
    property real demoSocRight: 100
    property real demoPhase: 0

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
            text: root.demoSweepEnabled
                  ? "UI SWEEP TEST"
                  : (dashboardController.canOnline ? "CAN Online" : "CAN Offline")
            color: root.demoSweepEnabled
                   ? "#87d88d"
                   : (dashboardController.canOnline ? "#87d88d" : "#ff7b7b")
            font.pixelSize: 16
            font.bold: true
        }
    }

    Speedometer {
        id: speedometer
        x: 238
        y: 73
        demoMode: root.demoSweepEnabled
        demoSpeedMph: root.demoSpeedMph
        onLogoPressed: root.vehicleExplorerRequested()
    }

    BatteryGauge {
        id: batteryGaugeLeft
        x: 24
        y: 66
        useRightMotor: false
        demoMode: root.demoSweepEnabled
        demoLevel: root.demoSocLeft
    }

    CurrentDraw {
        id: currentDrawLeft
        x: 24
        y: 116
        useRightMotor: false
        demoMode: root.demoSweepEnabled
        demoLevel: root.demoCurrentLeft
    }

    BatteryGauge {
        id: batteryGaugeRight
        x: 626
        y: 66
        useRightMotor: true
        demoMode: root.demoSweepEnabled
        demoLevel: root.demoSocRight
    }

    CurrentDraw {
        id: currentDrawRight
        x: 626
        y: 116
        useRightMotor: true
        demoMode: root.demoSweepEnabled
        demoLevel: root.demoCurrentRight
    }

    Timer {
        id: demoSweepTimer
        running: root.demoSweepEnabled
        repeat: true
        interval: 40
        onTriggered: {
            root.demoPhase += 0.045
            root.demoSpeedMph = (Math.sin(root.demoPhase) * 0.5 + 0.5) * 25.0
            root.demoCurrentLeft = (Math.sin(root.demoPhase * 1.6 + 0.8) * 0.5 + 0.5) * 95.0
            root.demoCurrentRight = (Math.sin(root.demoPhase * 1.25 + 2.1) * 0.5 + 0.5) * 95.0
            root.demoSocLeft = (Math.sin(root.demoPhase * 0.35 + 0.4) * 0.5 + 0.5) * 80.0 + 20.0
            root.demoSocRight = (Math.sin(root.demoPhase * 0.32 + 1.9) * 0.5 + 0.5) * 75.0 + 25.0
        }
    }

    Park {
        id: park
        x: 504
        y: 414
        width: 88
        height: 32
    }

    Lights {
        id: lights
        x: 652
        y: 414
        width: 88
        height: 32
    }

    Settings {
        id: settings
        x: 60
        y: 414
        width: 88
        height: 32
        onSettingsPressed: root.settingsRequested()
    }

    Lock {
        id: lock
        x: 208
        y: 414
        width: 88
        height: 32
    }

    DirectionButton {
        id: directionButton
        x: 356
        y: 414
        width: 88
        height: 32
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
