

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
import Qt5Compat.GraphicalEffects
import QMLCDash

Item {
    id: root
    width: parent.width * 0.38
    height: width
    property int speed: 0

       Connections {
           target: dashboardController
           function onLeftRpmChanged(rpm){
               root.speed = rpm; // value from MotorDataProcessor via DashboardController
           }
       }
    Rectangle {
        id: speedometer
        // scale width/height with window
        anchors.fill: parent
        radius: width/2
        gradient: Gradient {
            GradientStop {
                position: 0
                color: "#3f2626"
            }

            GradientStop {
                position: 1
                color: "#000000"
            }
            orientation: Gradient.Vertical
        }

        Image {
                id: ramsLogo
                //visible: DashboardController.locked
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
                source: "../images/CSU-Ram-Rev.png"
                mipmap: true
                anchors.fill: parent

            }

        ColorOverlay {
                anchors.fill: ramsLogo
                source: ramsLogo
                color: "darkgoldenrod" // Change this color dynamically
            }
/*
        Colorize {
            anchors.fill: ramsLogo
            source: ramsLogo
            //hue: 0.9 // Range 0.0 - 1.0
            saturation: 0.8
            lightness: 0.0
        }
*/
        Text {
            id: speed
            anchors.horizontalCenter: parent.horizontalCenter
            y: parent.height * 0.4
            width: 67
            color: "#eaeaea"
            //text: qsTr("0")
            text: qsTr("%1").arg(root.speed)
            font.pixelSize: 50
            horizontalAlignment: Text.AlignHCenter
        }

        Text {
            id: text1
            anchors.horizontalCenter: parent.horizontalCenter
            y: parent.height * 0.6
            color: "#eaeaea"
            text: qsTr("rpm")
            font.pixelSize: 24
        }
    }
}
