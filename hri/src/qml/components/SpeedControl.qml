import QtQuick
import QtQuick.Controls.Basic
import com.hri.app

// 速度控件：触屏友好离散按钮 ±56x56 (原 48x48)，环高 148 (原 132)
Item {
    id: root

    property real value: 1.5
    property real minValue: 0.5
    property real maxValue: 3.0
    property real step: 0.1
    property color accent: Theme.accent
    signal valueCommitted(real value)

    implicitWidth: 376
    implicitHeight: 148

    function clamp(v) {
        return Math.min(root.maxValue, Math.max(root.minValue, v))
    }

    readonly property real ratio: Math.max(0, Math.min(1, (value - minValue) / (maxValue - minValue)))

    Row {
        anchors.centerIn: parent
        spacing: 28

        // − 按钮 56x56，圆形
        Button {
            id: minusBtn
            width: 56; height: 56
            anchors.verticalCenter: parent.verticalCenter
            hoverEnabled: false
            onClicked: root.valueCommitted(root.clamp(root.value - root.step))

            background: Rectangle {
                radius: width / 2
                color: minusBtn.down ? Theme.withAlpha(root.accent, 0.22) : Theme.glass
                border.width: 1
                border.color: Theme.hairlineStrong
                Behavior on color { ColorAnimation { duration: Theme.msColor } }
                scale: minusBtn.down ? 0.92 : 1.0
                Behavior on scale { NumberAnimation { duration: Theme.msPress; easing.type: Easing.OutQuad } }
            }
            contentItem: Text {
                text: "−"
                font.pixelSize: 30
                color: Theme.textPrimary
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        Item {
            width: 148; height: 148
            anchors.verticalCenter: parent.verticalCenter

            Canvas {
                id: dial
                anchors.fill: parent
                antialiasing: true

                onWidthChanged: requestPaint()
                onHeightChanged: requestPaint()
                Connections {
                    target: root
                    function onValueChanged() { dial.requestPaint() }
                    function onAccentChanged() { dial.requestPaint() }
                }

                onPaint: {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)

                    var cx = width / 2, cy = height / 2
                    var r = width / 2 - 8
                    var start = Math.PI * 0.75
                    var span = Math.PI * 1.5
                    var end = start + span * root.ratio

                    ctx.lineCap = "round"

                    ctx.beginPath()
                    ctx.arc(cx, cy, r, start, start + span, false)
                    ctx.strokeStyle = "rgba(255,255,255,0.10)"
                    ctx.lineWidth = 5
                    ctx.stroke()

                    if (root.ratio > 0.001) {
                        ctx.beginPath()
                        ctx.arc(cx, cy, r, start, end, false)
                        ctx.shadowColor = Theme.css(root.accent, 0.55)
                        ctx.shadowBlur = 14
                        ctx.strokeStyle = Theme.css(Theme.accentSoft, 0.95)
                        ctx.lineWidth = 6
                        ctx.stroke()
                        ctx.shadowBlur = 0

                        var tx = cx + Math.cos(end) * r
                        var ty = cy + Math.sin(end) * r
                        ctx.beginPath()
                        ctx.arc(tx, ty, 5, 0, Math.PI * 2)
                        ctx.fillStyle = "#FFFFFF"
                        ctx.shadowColor = Theme.css(root.accent, 0.9)
                        ctx.shadowBlur = 12
                        ctx.fill()
                        ctx.shadowBlur = 0
                    }
                }
            }

            Column {
                anchors.centerIn: parent
                spacing: 0

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: root.value.toFixed(1)
                    font.pixelSize: 40
                    font.bold: true
                    color: Theme.textPrimary
                }

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "m/s"
                    font.pixelSize: 12
                    font.letterSpacing: 2
                    color: Theme.textTertiary
                }
            }
        }

        // + 按钮 56x56
        Button {
            id: plusBtn
            width: 56; height: 56
            anchors.verticalCenter: parent.verticalCenter
            hoverEnabled: false
            onClicked: root.valueCommitted(root.clamp(root.value + root.step))

            background: Rectangle {
                radius: width / 2
                color: plusBtn.down ? Theme.withAlpha(root.accent, 0.22) : Theme.glass
                border.width: 1
                border.color: Theme.hairlineStrong
                Behavior on color { ColorAnimation { duration: Theme.msColor } }
                scale: plusBtn.down ? 0.92 : 1.0
                Behavior on scale { NumberAnimation { duration: Theme.msPress; easing.type: Easing.OutQuad } }
            }
            contentItem: Text {
                text: "+"
                font.pixelSize: 30
                color: Theme.textPrimary
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
}
