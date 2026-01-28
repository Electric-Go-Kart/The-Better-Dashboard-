

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
//import QtQuick.Studio.DesignEffects

Item {
    id: batteryGauge
    width: 20
    height: 400
    property real level: 100   // starts at 100%

        Connections {
            target: dashboardController
            onLeftSocChanged: {
                batteryGauge.level = soc; // updated directly from MotorDataProcessor via DashboardController
            }
        }

    Rectangle {
        id: frame
        radius: 7
        anchors.fill: parent
        gradient: Gradient {
            GradientStop {
                position: 0
                color: "#700505"
            }

            GradientStop {
                position: 1
                color: "#000000"
            }
            orientation: Gradient.Vertical
        }

        Rectangle {
            id: fillBar
            x: 0
            width: 20
            //height: dashboardcontroller.charge
            //color: "#0f3704"
            anchors.bottom: parent.bottom
            gradient: Gradient {
                GradientStop {
                    position: 0
                    color: "#0d3716"
                }

                GradientStop {
                    position: 1
                    color: "#000000"
                }
                orientation: Gradient.Vertical
            }

            height: (parent.height - 8) * (batteryGauge.level / 100)
            radius: 7

            Behavior on height {
                NumberAnimation {
                    duration: 300
                    easing.type: Easing.OutQuad
                }
            }
        }

        Text {
            anchors.centerIn: parent
            text: batteryGauge.level + "%"
            color: "white"
            font.pixelSize: 20
            rotation: 90
            font.bold: true
        }
    }
}
