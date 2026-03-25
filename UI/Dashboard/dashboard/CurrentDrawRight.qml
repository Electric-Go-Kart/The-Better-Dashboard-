

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
import QtQuick.Shapes
import QMLCDash

Item {
    id: currentDraw
    width: parent.width * 0.25
    height: width
    property real level: 0.2   // starts at 100%

        Connections {
            target: dashboardController
            function onRightCurrentChanged(current){
                currentDraw.level = current; // updated directly from MotorDataProcessor via DashboardController
            }
        }

        Rectangle {
               anchors.fill: parent
               color: "#111"
               radius: width/2
           }
        // Background track
            Shape {
                anchors.fill: parent

                ShapePath {
                    strokeWidth: 22
                    strokeColor: "#222"
                    fillColor: "transparent"

                    PathAngleArc {
                        centerX: width/2
                        centerY: height/2
                        radiusX: width/2 - 11
                        radiusY: height/2 - 11
                        startAngle: -220
                        sweepAngle: 260
                    }
                }
            }

            // Active current bar
            Shape {
                anchors.fill: parent

                ShapePath {
                    strokeWidth: 22
                    strokeColor: "#00ff88"   // FIX
                    fillColor: "transparent"
                    capStyle: ShapePath.RoundCap

                    PathAngleArc {
                        centerX: width/2
                        centerY: height/2
                        radiusX: width/2 - 11
                        radiusY: height/2 - 11
                        startAngle: -220
                        sweepAngle: level * 260
                    }
                }
            }

            // Center current level
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                y: parent.height * 0.25
                text: Math.round(level * 100) + " A"
                color: "white"
                font.pixelSize: 40
                font.bold: true
            }

            // Startup sweep animation
                SequentialAnimation {
                    running: true
                    loops: 1   // only run once at startup
                    NumberAnimation {
                        target: currentDraw
                        property: "level"
                        from: 0
                        to: 1
                        duration: 1200
                        easing.type: Easing.InOutQuad
                    }
                    NumberAnimation {
                        target: currentDraw
                        property: "level"
                        from: 1
                        to: 0
                        duration: 1200
                        easing.type: Easing.InOutQuad
                    }
                }


}
