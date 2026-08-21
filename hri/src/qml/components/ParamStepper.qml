import QtQuick
import QtQuick.Controls.Basic
import com.hri.app

// 参数步进器：触屏友好 label + 数值 + ± 按钮，用于调节 PID / 触发参数
Item {
    id: root

    property string label: ""
    property real value: 0
    property real minValue: 0
    property real maxValue: 100
    property real step: 0.1
    property int decimals: 2
    property color accent: Theme.accent
    signal valueCommitted(real value)

    implicitWidth: 320
    implicitHeight: 56

    function clamp(v) {
        return Math.min(root.maxValue, Math.max(root.minValue, v))
    }

    Row {
        anchors.fill: parent
        spacing: 10

        Text {
            width: 110
            anchors.verticalCenter: parent.verticalCenter
            text: root.label
            font.pixelSize: 16
            color: Theme.textSecondary
            elide: Text.ElideRight
            verticalAlignment: Text.AlignVCenter
        }

        Item {
            width: 90
            height: parent.height
            anchors.verticalCenter: parent.verticalCenter

            Text {
                anchors.centerIn: parent
                text: root.value.toFixed(root.decimals)
                font.pixelSize: 20
                font.bold: true
                color: Theme.textPrimary
            }
        }

        // − 按钮
        Button {
            id: minusBtn
            width: 52; height: 52
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
                font.pixelSize: 26
                color: Theme.textPrimary
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        // + 按钮
        Button {
            id: plusBtn
            width: 52; height: 52
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
                font.pixelSize: 26
                color: Theme.textPrimary
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
}
