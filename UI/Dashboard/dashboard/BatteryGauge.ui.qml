

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
//import QtQuick.Studio.DesignEffects

Item {
    id: batteryGauge
    width: 150
    height: 42
    property real level: 100   // starts at 100%
    property bool useRightMotor: false
    property bool demoMode: false
    property real demoLevel: 100

    function displayLevel() {
        return demoMode ? demoLevel : level
    }

        Connections {
            target: dashboardController
            onLeftSocChanged: {
                if (!batteryGauge.useRightMotor) {
                    batteryGauge.level = soc;
                }
            }
            onRightSocChanged: {
                if (batteryGauge.useRightMotor) {
                    batteryGauge.level = soc;
                }
            }
        }

    Rectangle {
        id: frame
        radius: 7
        anchors.fill: parent
        border.width: 1
        border.color: Theme.csuGold
        gradient: Gradient {
            GradientStop {
                position: 0
                color: "#17362f"
            }

            GradientStop {
                position: 1
                color: Theme.surface
            }
            orientation: Gradient.Vertical
        }

        Rectangle {
            id: fillBar
            x: 3
            height: parent.height - 6
            //height: dashboardcontroller.charge
            //color: "#0f3704"
            anchors.verticalCenter: parent.verticalCenter
            gradient: Gradient {
                GradientStop {
                    position: 0
                    color: "#4AA3FF"
                }

                GradientStop {
                    position: 1
                    color: "#1A3F6A"
                }
                orientation: Gradient.Horizontal
            }

            width: (parent.width - 6) * (Math.min(Math.max(batteryGauge.displayLevel(), 0), 100) / 100)
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
            text: (batteryGauge.displayLevel()).toFixed(1) + "%"
            color: Theme.textPrimary
            font.pixelSize: 16
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }
}
