

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
    id: currentDraw
    width: 150
    height: 42
    property real level: 0   // starts at 100%
    property bool useRightMotor: false
    property bool demoMode: false
    property real demoLevel: 0

    function displayLevel() {
        return demoMode ? demoLevel : level
    }

        Connections {
            target: dashboardController
            onLeftCurrentChanged: {
                if (!currentDraw.useRightMotor) {
                    currentDraw.level = current;
                }
            }
            onRightCurrentChanged: {
                if (currentDraw.useRightMotor) {
                    currentDraw.level = current;
                }
            }
        }

    Rectangle {
        id: frame
        color: Theme.surface
        radius: 7
        anchors.fill: parent
        border.width: 1
        border.color: Theme.csuGold

        Rectangle {
            id: fillBar
            x: 0
            height: parent.height
            //height: dashboardcontroller.charge
            //color: "#0f3704"
            anchors.left: parent.left
            anchors.verticalCenter: parent.verticalCenter
            gradient: Gradient {
                GradientStop {
                    position: 0
                    color: Theme.warning
                }

                GradientStop {
                    position: 0.72
                    color: "#C87726"
                }

                GradientStop {
                    position: 0.9
                    color: "#8A4618"
                }

                GradientStop {
                    position: 1
                    color: "#5C2A0E"
                }
                orientation: Gradient.Horizontal
            }

            width: (parent.width - 6) * (Math.min(Math.max(currentDraw.displayLevel(), 0), 100) / 100)
            radius: 7

            Behavior on width {
                NumberAnimation {
                    duration: 300
                    easing.type: Easing.OutQuad
                }
            }
        }

        Text {
            anchors.centerIn: parent
            text: (currentDraw.displayLevel()).toFixed(1) + "A"
            color: Theme.textPrimary
            font.pixelSize: 18
            font.bold: true
        }
    }
}
