import QtQuick
import QtQuick.Controls
import "./dashboard/Theme.js" as Theme
import "./dashboard/KartGeometry.js" as Geo

Item {
    id: root
    width: 800
    height: 480

    readonly property real rangeM: Geo.rangeM
    readonly property real dangerM: Geo.dangerM
    readonly property real cautionM: Geo.cautionM
    readonly property real fovDeg: Geo.cameraFovDeg
    readonly property bool imperial: dashboardController.unitsSystem === "Imperial"

    function fmt(metres, approximate) {
        return Geo.formatDistance(metres, imperial, approximate)
    }

    // Kart sits below centre so the forward (camera) side gets most of the space.
    readonly property real kartX: bev.width / 2
    readonly property real kartY: bev.height * 0.62
    readonly property real pxPerM: (kartY - 12) / rangeM

    function goBack() {
        const stack = StackView.view
        if (stack && stack.depth > 1) {
            stack.pop()
        }
    }

    function zoneColor(distance) {
        if (distance < dangerM) return Theme.danger
        if (distance < cautionM) return Theme.warning
        return Theme.success
    }

    Rectangle {
        anchors.fill: parent
        color: Theme.surface
    }

    Rectangle {
        id: banner
        x: 0
        y: 0
        width: parent.width
        height: 72
        color: Theme.surfaceRaised
        border.width: 2
        border.color: Theme.csuGold

        Text {
            anchors.left: parent.left
            anchors.leftMargin: 24
            anchors.verticalCenter: parent.verticalCenter
            text: "What The Kart Sees"
            color: Theme.textPrimary
            font.pixelSize: 34
            font.bold: true
        }

        Rectangle {
            id: closestBadge
            anchors.right: backButton.left
            anchors.rightMargin: 16
            anchors.verticalCenter: parent.verticalCenter
            width: 170
            height: 46
            radius: 10
            color: "#0f1c18"
            border.width: 2
            border.color: detectionModel.closestDistance >= 0 ? root.zoneColor(detectionModel.closestDistance) : Theme.csuGold

            Text {
                anchors.centerIn: parent
                text: detectionModel.closestDistance >= 0
                      ? "Closest " + root.fmt(detectionModel.closestDistance, false)
                      : "Clear"
                color: Theme.textPrimary
                font.pixelSize: 20
                font.bold: true
            }
        }

        Button {
            id: backButton
            anchors.right: parent.right
            anchors.rightMargin: 24
            anchors.verticalCenter: parent.verticalCenter
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
    }

    Item {
        id: bev
        x: 0
        y: banner.height
        width: parent.width
        height: parent.height - banner.height
        clip: true

        // Static backdrop: camera field of view, range rings, zone colours.
        Canvas {
            anchors.fill: parent
            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                const cx = root.kartX
                const cy = root.kartY
                const s = root.pxPerM

                ctx.fillStyle = "#12221d"
                ctx.beginPath()
                ctx.moveTo(cx, cy)
                const half = root.fovDeg / 2 * Math.PI / 180
                ctx.arc(cx, cy, root.rangeM * s, -Math.PI / 2 - half, -Math.PI / 2 + half)
                ctx.closePath()
                ctx.fill()

                const rings = [root.dangerM, root.cautionM, root.rangeM]
                const colors = ["#ff7b7b", "#f2bf63", "#3b524a"]
                ctx.lineWidth = 1
                for (let i = 0; i < rings.length; ++i) {
                    ctx.strokeStyle = colors[i]
                    ctx.globalAlpha = i < 2 ? 0.55 : 1.0
                    ctx.beginPath()
                    ctx.arc(cx, cy, rings[i] * s, 0, Math.PI * 2)
                    ctx.stroke()
                }
                ctx.globalAlpha = 1.0

                ctx.strokeStyle = "#5f7f72"
                ctx.lineWidth = 1.5
                const reach = root.rangeM * s
                ctx.beginPath()
                ctx.moveTo(cx + Math.sin(half) * reach, cy - Math.cos(half) * reach)
                ctx.lineTo(cx, cy)
                ctx.lineTo(cx - Math.sin(half) * reach, cy - Math.cos(half) * reach)
                ctx.stroke()

                // Centre line splits the left and right camera regions.
                ctx.setLineDash([6, 5])
                ctx.beginPath()
                ctx.moveTo(cx, cy)
                ctx.lineTo(cx, cy - reach)
                ctx.stroke()
                ctx.setLineDash([])

                ctx.fillStyle = "#87a396"
                ctx.font = "12px sans-serif"
                ctx.fillText(root.fmt(root.dangerM, false), cx + 4, cy - root.dangerM * s - 3)
                ctx.fillText(root.fmt(root.cautionM, false), cx + 4, cy - root.cautionM * s - 3)
                ctx.fillText(root.fmt(root.rangeM, false), cx + 4, cy - root.rangeM * s - 3)
                ctx.font = "bold 13px sans-serif"
                ctx.fillText("L CAM", cx - Math.sin(half * 0.8) * reach * 0.93 - 8, cy - Math.cos(half * 0.8) * reach * 0.93 + 14)
                ctx.fillText("R CAM", cx + Math.sin(half * 0.8) * reach * 0.93 - 24, cy - Math.cos(half * 0.8) * reach * 0.93 + 14)
            }

            Connections {
                target: root
                function onImperialChanged() { requestPaint() }
            }
        }

        // Dim raw lidar returns behind the icons.
        Canvas {
            id: lidarCanvas
            anchors.fill: parent

            Connections {
                target: lidarFeed
                function onPointsChanged() { lidarCanvas.requestPaint() }
            }

            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                ctx.fillStyle = "#4d6f62"
                const pts = lidarFeed.points
                const s = root.pxPerM
                for (let i = 0; i < pts.length; ++i) {
                    ctx.fillRect(root.kartX + pts[i].y * s - 1, root.kartY - pts[i].x * s - 1, 2, 2)
                }
            }
        }

        // The kart body, sized from KartGeometry.js; the lidar origin is (kartX, kartY).
        Rectangle {
            x: root.kartX - Geo.kartWidthM / 2 * root.pxPerM
            y: root.kartY - Geo.kartFrontM() * root.pxPerM
            width: Geo.kartWidthM * root.pxPerM
            height: Geo.kartLengthM * root.pxPerM
            radius: 4
            color: Theme.csuGold
            opacity: 0.9
        }

        Repeater {
            model: detectionModel

            delegate: Item {
                id: marker
                required property int objectId
                required property string objectClass
                required property real posX
                required property real posY
                required property real distance
                required property string source
                required property real velX
                required property real velY

                readonly property bool unknown: objectClass === "unknown"
                readonly property color tint: unknown ? "#cfd8d4" : root.zoneColor(distance)

                x: root.kartX + posY * root.pxPerM - 14
                y: root.kartY - posX * root.pxPerM - 14
                width: 28
                height: 28
                visible: distance <= root.rangeM

                Behavior on x { NumberAnimation { duration: 40 } }
                Behavior on y { NumberAnimation { duration: 40 } }

                readonly property real speed: Math.hypot(velX, velY)
                readonly property bool seenByCamera: source === "fused" || source === "camera"

                // Motion arrow: points the way the object is moving, longer when faster.
                Item {
                    anchors.centerIn: parent
                    width: 1
                    height: 1
                    visible: marker.speed > 0.3
                    rotation: Math.atan2(marker.velY, marker.velX) * 180 / Math.PI

                    Rectangle {
                        readonly property real len: 6 + Math.min(marker.speed, 8) * 3
                        x: -1.5
                        y: -(16 + len)
                        width: 3
                        height: len
                        color: marker.tint
                    }
                    Rectangle {
                        x: -5
                        y: -(16 + 6 + Math.min(marker.speed, 8) * 3) - 4
                        width: 10
                        height: 10
                        rotation: 45
                        color: marker.tint
                    }
                }

                // Pulse when close
                Rectangle {
                    anchors.centerIn: parent
                    width: 28
                    height: 28
                    radius: 14
                    color: "transparent"
                    border.width: 2
                    border.color: Theme.danger
                    visible: marker.distance < root.dangerM
                    SequentialAnimation on scale {
                        running: marker.distance < root.dangerM
                        loops: Animation.Infinite
                        NumberAnimation { from: 1.0; to: 1.8; duration: 700 }
                    }
                    SequentialAnimation on opacity {
                        running: marker.distance < root.dangerM
                        loops: Animation.Infinite
                        NumberAnimation { from: 0.9; to: 0.0; duration: 700 }
                    }
                }

                DetectionIcon {
                    anchors.fill: parent
                    objectClass: marker.objectClass
                    tint: marker.tint
                }

                // Camera badge: classified by the camera, not lidar-only.
                Rectangle {
                    visible: marker.seenByCamera
                    x: parent.width - 8
                    y: -4
                    width: 12
                    height: 9
                    radius: 2
                    color: Theme.csuGold
                    border.width: 1
                    border.color: "#0f1c18"
                    Rectangle {
                        anchors.centerIn: parent
                        width: 4
                        height: 4
                        radius: 2
                        color: "#0f1c18"
                    }
                }

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: parent.bottom
                    anchors.topMargin: 1
                    text: root.fmt(marker.distance, marker.source === "camera")
                    color: Theme.textPrimary
                    font.pixelSize: 12
                    font.bold: true
                    style: Text.Outline
                    styleColor: "#0f1c18"
                }
            }
        }

        Text {
            anchors.left: parent.left
            anchors.leftMargin: 14
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 10
            visible: !lidarFeed.streaming && !detectionModel.demo
            text: "No sensor data"
            color: Theme.textMuted
            font.pixelSize: 18
        }
    }
}
