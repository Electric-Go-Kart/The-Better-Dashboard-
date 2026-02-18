

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
    width: Screen.width //800
    height: Screen.height //480

    Speedometer {
        id: speedometer
        x: 225
        y: 65
    }

    DirectionButton {
        id: directionButton
        x: 606
        y: 65
    }

    BatteryGauge {
        id: batteryGauge
        x: 743
        y: 40
    }

    CurrentDraw {
        id: currentDraw
        x: 37
        y: 40
    }

    Park {
        id: park
        x: 606
        y: 130
    }

    Lights {
        id: lights
        x: 606
        y: 194
    }

    ShutDown {
        id: shutDown
        x: 94
        y: 65
    }

    Settings {
        id: settings
        x: 94
        y: 130
    }

    Lock {
        id: lock
        x: 94
        y: 194
    }
}
