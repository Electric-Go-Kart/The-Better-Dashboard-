import QtQuick
import QtQuick.VirtualKeyboard
import QtQuick.Window
import QtQuick.Controls
import "./UI/Dashboard/dashboard"

// Defines main app window
Window {
    id: window
    width: 800
    height: 480
    visible: true

    Dashboard {
        id: dashboardScreen
        anchors.centerIn: parent
    }

    // VirtualKeyboard appears when text field is active
    InputPanel {
        id: inputPanel
        z: 99
        x: 0
        y: window.height
        width: window.width

        states: State {
            name: "visible"
            when: inputPanel.active
            PropertyChanges {
                target: inputPanel
                y: window.height - inputPanel.height
            }
        }
        // Transition animation
        transitions: Transition {
            from: ""
            to: "visible"
            reversible: true
            ParallelAnimation {
                NumberAnimation {
                    properties: "y"
                    duration: 250
                    easing.type: Easing.InOutQuad
                }
            }
        }
    }
}
