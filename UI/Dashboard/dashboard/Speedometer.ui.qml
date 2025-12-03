

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick
import QtQuick.Controls
Item {
    id: root
    width: 350
    height: 350
    Rectangle {
        id: speedometer
        width: 350
        height: 350
        radius: 175
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

        Text {
            id: speed
            x: 142
            y: 142
            width: 67
            color: "#eaeaea"
            text: qsTr("0")
            font.pixelSize: 50
            horizontalAlignment: Text.AlignHCenter
        }

        Text {
            id: text1
            x: 154
            y: 225
            color: "#eaeaea"
            text: qsTr("mph")
            font.pixelSize: 24
        }
    }
}
