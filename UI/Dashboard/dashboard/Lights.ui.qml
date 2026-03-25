

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
import QMLCDash
import "Theme.js" as Theme

Item {
    id: root
    width: 100
    height: 40
    property bool lightsEnabled: dashboardController.lightsEnabled

    Button {
        id: lightsButton
        visible: true
        opacity: 1
        text: root.lightsEnabled ? "Lights ON" : "Lights OFF"
        anchors.fill: parent
        rotation: 0
        flat: false
        onClicked: dashboardController.toggleLights()

        background: Rectangle {
            id: bg
            radius: 10
            color: root.lightsEnabled ? Theme.csuGreenBright : Theme.panelDark
            border.width: 2
            border.color: Theme.csuGold

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
    }
}
