import QtQuick
import QtQuick.Controls
import QMLCDash
import "./dashboard/Theme.js" as Theme

Item {
    id: root
    width: 800
    height: 480
    property real brightnessValue: dashboardController.uiBrightness
    property string selectedUnits: dashboardController.unitsSystem
    property string selectedMode: dashboardController.driveMode
    property var navStack: null
    property bool showCanDebug: canController.debugFramesEnabled

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
            spacing: 14

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
                        dashboardController.setUiBrightness(value)
                        root.resetInactivityTimer()
                    }
                }

                Text {
                    text: Math.round(dashboardController.uiBrightness * 100) + "%"
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
                    palette.button: "#FFFFFF"
                    palette.base: "#FFFFFF"
                    palette.text: "#1A1A1A"
                    contentItem: Text {
                        leftPadding: 10
                        rightPadding: unitsCombo.indicator.width + unitsCombo.spacing
                        text: unitsCombo.displayText
                        font: unitsCombo.font
                        color: "#1A1A1A"
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    background: Rectangle {
                        radius: 6
                        color: "#FFFFFF"
                        border.width: 1
                        border.color: Theme.csuGold
                    }
                    onActivated: {
                        root.selectedUnits = currentText
                        dashboardController.setUnitsSystem(currentText)
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
                    palette.button: "#FFFFFF"
                    palette.base: "#FFFFFF"
                    palette.text: "#1A1A1A"
                    contentItem: Text {
                        leftPadding: 10
                        rightPadding: modeCombo.indicator.width + modeCombo.spacing
                        text: modeCombo.displayText
                        font: modeCombo.font
                        color: "#1A1A1A"
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    background: Rectangle {
                        radius: 6
                        color: "#FFFFFF"
                        border.width: 1
                        border.color: Theme.csuGold
                    }
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

            Row {
                spacing: 12

                Button {
                    width: 190
                    height: 42
                    text: root.showCanDebug ? "Hide CAN Debug" : "Show CAN Debug"
                    onClicked: {
                        root.showCanDebug = !root.showCanDebug
                        canController.debugFramesEnabled = root.showCanDebug
                        if (!root.showCanDebug) {
                            canController.clearDebugFrameLog()
                        }
                        root.resetInactivityTimer()
                    }
                    background: Rectangle {
                        radius: 8
                        color: parent.down ? Theme.csuGreenBright : Theme.csuGreen
                        border.width: 1
                        border.color: Theme.csuGold
                    }
                    contentItem: Text {
                        text: parent.text
                        color: Theme.textPrimary
                        font.pixelSize: 16
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }

                Button {
                    width: 150
                    height: 42
                    text: "Clear Frame Log"
                    enabled: root.showCanDebug
                    onClicked: {
                        canController.clearDebugFrameLog()
                        root.resetInactivityTimer()
                    }
                    background: Rectangle {
                        radius: 8
                        color: parent.enabled ? (parent.down ? "#7E4C19" : Theme.warning) : "#4D4D4D"
                        border.width: 1
                        border.color: Theme.csuGold
                    }
                    contentItem: Text {
                        text: parent.text
                        color: parent.enabled ? "#1E1307" : "#B8B8B8"
                        font.pixelSize: 16
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: root.showCanDebug ? 120 : 0
                visible: root.showCanDebug
                radius: 10
                color: Theme.panelDark
                border.width: 1
                border.color: Theme.csuGold
                clip: true

                Behavior on height {
                    NumberAnimation {
                        duration: 180
                        easing.type: Easing.OutQuad
                    }
                }

                ScrollView {
                    anchors.fill: parent
                    anchors.margins: 10
                    clip: true

                    TextArea {
                        readOnly: true
                        text: canController.debugFrameLog.length > 0
                              ? canController.debugFrameLog
                              : "Incoming CAN frames will appear here while debug is enabled."
                        color: Theme.textPrimary
                        wrapMode: Text.NoWrap
                        font.family: "Menlo"
                        font.pixelSize: 13
                        background: null
                        selectByMouse: true
                    }
                }
            }

            Item {
                width: 1
                height: 8
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
