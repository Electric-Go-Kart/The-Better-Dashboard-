import QtQuick
//import GoKartDash
import "./dashboard"

Window {
    width: 800
    height: 480
    // Removes the title bar and window frame
    //flags: Qt.FramelessWindowHint

    visible: true
    title: "GoKartDash"


    Dashboard{
        id: mainScreen
    }

    //Screen01 {
    //    id: mainScreen
    //}

}

