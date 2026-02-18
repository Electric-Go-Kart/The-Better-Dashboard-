import QtQuick

Rectangle{
    id: circle
    width: 300
    height: 300
    radius: width / 2   // makes it a perfect circle
    color: "black"
    anchors.centerIn: parent

    Text {
        property bool open: true

        id: mph
        color: "#FFFFFF"
        text: "55"//DashboardController.speed
        font.pointSize: 40
        anchors.verticalCenter: parent.verticalCenter
        styleColor: "#FFFFFF"
        anchors.horizontalCenter: parent.horizontalCenter
        font.family: "Haettenschweiler"

        Text {
            id: mphLabel
            text: "mph"
            anchors.top: parent.bottom
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.topMargin: 10
            font.family: "Haettenschweiler"
            color: "#FFFFFF"
            styleColor: "#FFFFFF"
        }

    }
}




