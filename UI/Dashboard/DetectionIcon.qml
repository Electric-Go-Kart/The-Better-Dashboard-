import QtQuick
import "./dashboard/Theme.js" as Theme

// Small flat icon for one detected object class, drawn from primitives (no image assets).
Item {
    id: icon
    property string objectClass: "unknown"
    property color tint: Theme.textPrimary
    width: 28
    height: 28

    Rectangle {
        anchors.fill: parent
        radius: width / 2
        color: "#cc0f1c18"
        border.width: 2
        border.color: icon.tint
    }

    // person: head and shoulders
    Item {
        anchors.centerIn: parent
        visible: icon.objectClass === "person"
        Rectangle { x: -3; y: -9; width: 6; height: 6; radius: 3; color: icon.tint }
        Rectangle { x: -5; y: -2; width: 10; height: 9; radius: 4; color: icon.tint }
    }

    // vehicle: car from above with a windscreen
    Item {
        anchors.centerIn: parent
        visible: icon.objectClass === "vehicle"
        Rectangle { x: -5; y: -9; width: 10; height: 18; radius: 3; color: icon.tint }
        Rectangle { x: -3; y: -5; width: 6; height: 4; radius: 1; color: "#0f1c18" }
    }

    // cone: base plus stem
    Item {
        anchors.centerIn: parent
        visible: icon.objectClass === "cone"
        Rectangle { x: -7; y: 4; width: 14; height: 4; radius: 1; color: icon.tint }
        Rectangle { x: -4; y: -2; width: 8; height: 7; color: icon.tint }
        Rectangle { x: -2; y: -8; width: 4; height: 7; color: icon.tint }
    }

    // bike: two wheels and a frame
    Item {
        anchors.centerIn: parent
        visible: icon.objectClass === "bike"
        Rectangle { x: -4; y: -10; width: 8; height: 8; radius: 4; color: "transparent"; border.width: 2; border.color: icon.tint }
        Rectangle { x: -4; y: 2; width: 8; height: 8; radius: 4; color: "transparent"; border.width: 2; border.color: icon.tint }
        Rectangle { x: -1; y: -4; width: 2; height: 8; color: icon.tint }
    }

    // pole/barrier: striped bar on two posts
    Item {
        anchors.centerIn: parent
        visible: icon.objectClass === "pole/barrier"
        Rectangle { x: -8; y: -3; width: 16; height: 6; radius: 1; color: icon.tint }
        Rectangle { x: -3; y: -2; width: 2; height: 4; color: "#0f1c18" }
        Rectangle { x: 2; y: -2; width: 2; height: 4; color: "#0f1c18" }
        Rectangle { x: -7; y: 3; width: 2; height: 6; color: icon.tint }
        Rectangle { x: 5; y: 3; width: 2; height: 6; color: icon.tint }
    }

    Text {
        anchors.centerIn: parent
        visible: ["person", "vehicle", "cone", "bike", "pole/barrier"].indexOf(icon.objectClass) < 0
        text: "?"
        color: icon.tint
        font.pixelSize: 20
        font.bold: true
    }
}
