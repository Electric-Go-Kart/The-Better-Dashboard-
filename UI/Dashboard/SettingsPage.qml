import QtQuick
import QtQuick.Controls
import QMLCDash
import "./dashboard/Theme.js" as Theme

Item {
    id: root
    width: 800
    height: 480
    property real brightnessValue: 0.8
    property string selectedUnits: "Metric"
    property string selectedMode: dashboardController.driveMode
    property var navStack: null

    function resetInactivityTimer() {
        inactivityTimer.restart()
    }

    function returnToMainDash() {
        const stack = root.navStack ? root.navStack : StackView.view
        if (stack && stack.depth > 1) {
            stack.pop()
        }
    }

    Component.onCompleted: {
        root.navStack = StackView.view
    }

    Rectangle {
        anchors.fill: parent
        color: Theme.surface
    }

    Rectangle {
        x: 24
        y: 20
        width: 752
        height: 440
        radius: 14
        color: Theme.surfaceRaised
        border.width: 2
        border.color: Theme.csuGold

        Column {
            anchors.fill: parent
            anchors.margins: 24
            spacing: 18

            Text {
                text: "Settings"
                color: Theme.textPrimary
                font.pixelSize: 34
                font.bold: true
            }

            Row {
                spacing: 16
                width: parent.width

                Text {
                    width: 220
                    text: "Brightness"
                    color: Theme.textPrimary
                    font.pixelSize: 22
                }

                Slider {
                    id: brightnessSlider
                    width: 420
                    from: 0.2
                    to: 1.0
                    value: root.brightnessValue
                    onMoved: {
                        root.brightnessValue = value
                        root.resetInactivityTimer()
                    }
                }

                Text {
                    text: Math.round(root.brightnessValue * 100) + "%"
                    color: Theme.textMuted
                    font.pixelSize: 20
                }
            }

            Row {
                spacing: 16
                width: parent.width

                Text {
                    width: 220
                    text: "Units"
                    color: Theme.textPrimary
                    font.pixelSize: 22
                }

                ComboBox {
                    id: unitsCombo
                    width: 220
                    model: ["Metric", "Imperial"]
                    currentIndex: root.selectedUnits === "Metric" ? 0 : 1
                    onActivated: {
                        root.selectedUnits = currentText
                        root.resetInactivityTimer()
                    }
                }
            }

            Row {
                spacing: 16
                width: parent.width

                Text {
                    width: 220
                    text: "Default Drive Mode"
                    color: Theme.textPrimary
                    font.pixelSize: 22
                }

                ComboBox {
                    id: modeCombo
                    width: 220
                    model: ["Eco", "Normal", "Sport"]
                    currentIndex: model.indexOf(root.selectedMode)
                    onActivated: {
                        root.selectedMode = currentText
                        dashboardController.setDriveMode(currentText)
                        root.resetInactivityTimer()
                    }
                }
            }

            Row {
                spacing: 12
                Text {
                    text: "CAN Status:"
                    color: Theme.textPrimary
                    font.pixelSize: 21
                    font.bold: true
                }
                Text {
                    text: dashboardController.canStatus
                    color: dashboardController.canOnline ? Theme.success : Theme.danger
                    font.pixelSize: 21
                    font.bold: true
                }
            }

            Item {
                width: 1
                height: 24
            }

            Button {
                text: "Return To Main Dash"
                width: 280
                height: 58
                onClicked: root.returnToMainDash()
                background: Rectangle {
                    radius: 10
                    color: Theme.csuGreen
                    border.width: 2
                    border.color: Theme.csuGold
                }
                contentItem: Text {
                    text: parent.text
                    color: Theme.textPrimary
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 22
                    font.bold: true
                }
            }
        }
    }

    TapHandler {
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchScreen
        onTapped: root.resetInactivityTimer()
    }

    Timer {
        id: inactivityTimer
        interval: 30000
        repeat: false
        running: true
        onTriggered: root.returnToMainDash()
    }
}
