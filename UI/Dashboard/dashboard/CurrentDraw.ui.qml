

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls

Item {
    id: currentDraw
    width: 20
    height: 400
    property real level: 0   // starts at 100%

        Connections {
            target: dashboardController
            onLeftCurrentChanged: {
                currentDraw.level = current; // updated directly from MotorDataProcessor via DashboardController
            }
        }

    Rectangle {
        id: frame
        color: "#030303"
        radius: 7
        anchors.fill: parent

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
                    color: "#d81505"
                }

                GradientStop {
                    position: 1
                    color: "#c65c03"
                }
                orientation: Gradient.Vertical
            }

            height: parent.height * (currentDraw.level / 40)
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
            text: currentDraw.level + "A"
            color: "white"
            font.pixelSize: 20
            rotation: 90
            font.bold: true
        }
    }
}
