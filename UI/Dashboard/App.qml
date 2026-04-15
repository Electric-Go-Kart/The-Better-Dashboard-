import QtQuick
import QtQuick.Controls
import "./dashboard"

Item {
    id: root
    width: 800
    height: 480
    property var stackView: null

    property string transientMessage: ""
    property bool transientVisible: false

    Rectangle {
        anchors.fill: parent
        color: "#0c1412"
    }

    Dashboard {
        id: mainScreen
        anchors.fill: parent
        onSettingsRequested: {
            const navStack = StackView.view ? StackView.view : root.stackView
            if (dashboardController.settingsAccessAllowed) {
                if (navStack) {
                    navStack.push("SettingsPage.qml")
                } else {
                    root.transientMessage = "Navigation unavailable"
                    root.transientVisible = true
                    transientMessageTimer.restart()
                }
            } else {
                root.transientMessage = "Settings available only at low speed"
                root.transientVisible = true
                transientMessageTimer.restart()
            }
        }
        onVehicleExplorerRequested: {
            const navStack = StackView.view ? StackView.view : root.stackView
            if (navStack) {
                navStack.push("VehicleExplorerPage.qml")
            } else {
                root.transientMessage = "Navigation unavailable"
                root.transientVisible = true
                transientMessageTimer.restart()
            }
        }
    }

    Rectangle {
        id: messageBanner
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 14
        width: Math.min(parent.width - 40, 460)
        height: 34
        radius: 8
        visible: root.transientMessage.length > 0 || opacity > 0
        opacity: root.transientVisible ? 1 : 0
        color: "#A30B1B"
        border.color: "#F7F7F7"
        border.width: 1

        Behavior on opacity {
            NumberAnimation {
                duration: 320
                easing.type: Easing.InOutQuad
            }
        }

        Text {
            anchors.centerIn: parent
            text: root.transientMessage
            color: "#F7F7F7"
            font.pixelSize: 16
            font.bold: true
        }
    }

    Timer {
        id: transientMessageTimer
        interval: 3000
        repeat: false
        onTriggered: {
            root.transientVisible = false
            clearTransientTextTimer.restart()
        }
    }

    Timer {
        id: clearTransientTextTimer
        interval: 360
        repeat: false
        onTriggered: root.transientMessage = ""
    }
}
