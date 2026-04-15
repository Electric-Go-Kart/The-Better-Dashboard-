import QtQuick
import QtQuick.Controls
import "./dashboard/Theme.js" as Theme

Item {
    id: root
    width: 800
    height: 480

    property var navStack: null
    property string selectedComponent: "brakes"

    readonly property var componentDetails: ({
        "brakes": {
            "title": "Brakes",
            "summary": "The braking system slows the kart and helps hold it in place when parked.",
            "details": "This section can be expanded later with hydraulic layout notes, parking behavior details, and maintenance guidance."
        },
        "motors": {
            "title": "Drive Motors",
            "summary": "The two drive motors convert electrical energy into wheel torque for propulsion.",
            "details": "This overview is intentionally descriptive only. It is a good place to explain motor placement, gearing, and left-right drive behavior."
        },
        "dashboard": {
            "title": "Dashboard",
            "summary": "The dashboard is the driver-facing interface for vehicle state, warnings, and controls.",
            "details": "Use this section to describe what the main driving screen shows and how the controls are organized for the driver."
        },
        "seat": {
            "title": "Seat",
            "summary": "The seat supports the driver and helps position weight correctly within the kart chassis.",
            "details": "This area can later include notes about mounting, ergonomics, and any safety considerations tied to seating position."
        },
        "controllers": {
            "title": "Motor Controllers",
            "summary": "The motor controllers interpret drive commands and regulate the electrical power sent to the motors.",
            "details": "This section is suited for controller responsibilities, protections, communication, and simplified system diagrams."
        }
    })

    function componentInfo(key) {
        return root.componentDetails[key] || root.componentDetails["brakes"]
    }

    function selectComponent(key) {
        root.selectedComponent = key
    }

    function goBack() {
        const stack = root.navStack ? root.navStack : StackView.view
        if (stack && stack.depth > 1) {
            stack.pop()
        }
    }

    Component.onCompleted: {
        root.navStack = StackView.view
    }

    Rectangle {
        anchors.fill: parent
        color: Theme.surface
    }

    Rectangle {
        x: 18
        y: 16
        width: 764
        height: 448
        radius: 16
        color: Theme.surfaceRaised
        border.width: 2
        border.color: Theme.csuGold
    }

    Text {
        x: 40
        y: 28
        text: "Vehicle Explorer"
        color: Theme.textPrimary
        font.pixelSize: 34
        font.bold: true
    }

    Text {
        x: 40
        y: 68
        text: "Tap a highlighted section on the simplified overhead go-kart view."
        color: Theme.textMuted
        font.pixelSize: 17
    }

    Button {
        id: backButton
        x: 622
        y: 28
        width: 136
        height: 46
        text: "Back"
        onClicked: root.goBack()

        background: Rectangle {
            radius: 10
            color: backButton.down ? Theme.csuGreenBright : Theme.csuGreen
            border.width: 2
            border.color: Theme.csuGold
        }

        contentItem: Text {
            text: backButton.text
            color: Theme.textPrimary
            font.pixelSize: 20
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    Rectangle {
        id: modelPanel
        x: 34
        y: 104
        width: 420
        height: 328
        radius: 14
        color: Theme.panelDark
        border.width: 1
        border.color: Theme.csuGold
    }

    Text {
        x: modelPanel.x + 18
        y: modelPanel.y + 14
        text: "Simplified Overhead Render"
        color: Theme.textPrimary
        font.pixelSize: 20
        font.bold: true
    }

    Item {
        id: chassisScene
        x: modelPanel.x + 24
        y: modelPanel.y + 58
        width: 372
        height: 230

        Rectangle {
            x: 114
            y: 30
            width: 150
            height: 146
            radius: 10
            color: "#21463b"
            border.width: 2
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 138
            y: 18
            width: 102
            height: 22
            radius: 8
            color: "#2d5c4d"
            border.width: 1
            border.color: "#d7c58d"
        }

        Rectangle {
            x: 145
            y: 4
            width: 88
            height: 28
            radius: 8
            color: root.selectedComponent === "dashboard" ? Theme.warning : Theme.csuGreenBright
            border.width: 2
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 144
            y: 66
            width: 90
            height: 62
            radius: 10
            color: root.selectedComponent === "seat" ? Theme.warning : "#355a4e"
            border.width: 2
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 156
            y: 132
            width: 60
            height: 28
            radius: 8
            color: root.selectedComponent === "controllers" ? Theme.warning : "#315d4f"
            border.width: 2
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 72
            y: 138
            width: 38
            height: 24
            radius: 8
            color: root.selectedComponent === "motors" ? Theme.warning : "#396d5a"
            border.width: 2
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 264
            y: 138
            width: 38
            height: 24
            radius: 8
            color: root.selectedComponent === "motors" ? Theme.warning : "#396d5a"
            border.width: 2
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 120
            y: 178
            width: 138
            height: 18
            radius: 8
            color: root.selectedComponent === "brakes" ? Theme.warning : "#5f2c16"
            border.width: 2
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 136
            y: 184
            width: 18
            height: 16
            radius: 6
            color: root.selectedComponent === "brakes" ? Theme.warning : "#82401e"
            border.width: 1
            border.color: Theme.csuGold
        }

        Rectangle {
            x: 224
            y: 184
            width: 18
            height: 16
            radius: 6
            color: root.selectedComponent === "brakes" ? Theme.warning : "#82401e"
            border.width: 2
            border.color: Theme.csuGold
        }

        Repeater {
            model: [
                { "x": 70, "y": 34 },
                { "x": 272, "y": 34 },
                { "x": 70, "y": 162 },
                { "x": 272, "y": 162 }
            ]

            delegate: Rectangle {
                x: modelData.x
                y: modelData.y
                width: 28
                height: 64
                radius: 8
                color: "#101816"
                border.width: 2
                border.color: "#6f7b77"
            }
        }

        Button {
            x: 137
            y: 0
            width: 104
            height: 42
            text: "Dashboard"
            onClicked: root.selectComponent("dashboard")
        }

        Button {
            x: 128
            y: 54
            width: 116
            height: 76
            text: "Seat"
            onClicked: root.selectComponent("seat")
        }

        Button {
            x: 142
            y: 126
            width: 88
            height: 42
            text: "Controller"
            onClicked: root.selectComponent("controllers")
        }

        Button {
            x: 20
            y: 124
            width: 92
            height: 52
            text: "Motor"
            onClicked: root.selectComponent("motors")
        }

        Button {
            x: 260
            y: 124
            width: 92
            height: 52
            text: "Motor"
            onClicked: root.selectComponent("motors")
        }

        Button {
            x: 109
            y: 168
            width: 160
            height: 38
            text: "Brakes"
            onClicked: root.selectComponent("brakes")
        }
    }

    Rectangle {
        id: quickSelectPanel
        x: 34
        y: 390
        width: 420
        height: 42
        radius: 10
        color: "#143028"
        border.width: 1
        border.color: Theme.csuGold
    }

    Row {
        x: quickSelectPanel.x + 10
        y: quickSelectPanel.y + 6
        spacing: 8

        Repeater {
            model: [
                { "key": "brakes", "label": "Brakes" },
                { "key": "motors", "label": "Motors" },
                { "key": "dashboard", "label": "Dashboard" },
                { "key": "seat", "label": "Seat" },
                { "key": "controllers", "label": "Controllers" }
            ]

            delegate: Button {
                width: 76
                height: 30
                text: modelData.label
                onClicked: root.selectComponent(modelData.key)

                background: Rectangle {
                    radius: 8
                    color: root.selectedComponent === modelData.key ? Theme.warning : Theme.csuGreen
                    border.width: 1
                    border.color: Theme.csuGold
                }

                contentItem: Text {
                    text: parent.text
                    color: root.selectedComponent === modelData.key ? "#2b1b08" : Theme.textPrimary
                    font.pixelSize: 15
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }
    }

    Rectangle {
        id: infoPanel
        x: 478
        y: 104
        width: 280
        height: 328
        radius: 14
        color: Theme.panelDark
        border.width: 1
        border.color: Theme.csuGold
    }

    Rectangle {
        x: infoPanel.x + 18
        y: infoPanel.y + 20
        width: 244
        height: 52
        radius: 10
        color: Theme.csuGreen
        border.width: 1
        border.color: Theme.csuGold
    }

    Text {
        x: infoPanel.x + 30
        y: infoPanel.y + 31
        width: 220
        text: root.componentInfo(root.selectedComponent).title
        color: Theme.textPrimary
        font.pixelSize: 26
        font.bold: true
        wrapMode: Text.WordWrap
    }

    Text {
        x: infoPanel.x + 22
        y: infoPanel.y + 92
        width: 236
        text: root.componentInfo(root.selectedComponent).summary
        color: Theme.textPrimary
        font.pixelSize: 18
        wrapMode: Text.WordWrap
    }

    Text {
        x: infoPanel.x + 22
        y: infoPanel.y + 176
        width: 236
        text: root.componentInfo(root.selectedComponent).details
        color: Theme.textMuted
        font.pixelSize: 16
        wrapMode: Text.WordWrap
    }

    Text {
        x: infoPanel.x + 22
        y: infoPanel.y + 286
        width: 236
        text: "This menu is informational only and does not change vehicle state."
        color: Theme.warning
        font.pixelSize: 15
        font.bold: true
        wrapMode: Text.WordWrap
    }
}
