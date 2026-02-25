

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
    width: 100
    height: 40

    Button {
        id: lightsButton
        visible: true
        // Temporary test variable — REMOVE later
        property bool fakeLights: true
        opacity: 1
        text: /*dashboardcontroller.Lights*/ fakeLights ? "Lights ON" : "Lights OFF"
        anchors.fill: parent
        rotation: 0
        flat: false

        background: Rectangle {
            id: bg
            radius: 10
            color: fakeLights ? "#2ecc71" : "#e74c3c" // green / red
            border.width: 2
            border.color: "#222"

            // Press feedback — slightly shrink on touch
            scale: lightsButton.down ? 0.95 : 1.0

            Behavior on scale {
                NumberAnimation {
                    duration: 80
                    easing.type: Easing.OutQuad
                }
            }

            // Smooth color transition when direction changes
            Behavior on color {
                ColorAnimation {
                    duration: 200
                    easing.type: Easing.InOutQuad
                }
            }
        }

        TapHandler {
            id: tapper
            onTapped: lightsButton.fakeLights = !lightsButton.fakeLights
        }
    }
}
