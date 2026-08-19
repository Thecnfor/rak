import QtQuick
import QtQuick.Controls.Basic
import Qt5Compat.GraphicalEffects
import com.hri.app

// 启动按钮：触摸屏二次确认 + 加大高度 64，最小热区 64x260
Button {
    id: control

    property string label: "启动"
    property color accent: Theme.accent
    property string armedText: "再次点击确认启动"
    signal confirmed()

    property bool armed: false
    text: armed ? armedText : label
    hoverEnabled: false

    Timer {
        id: disarmer
        interval: 4000        // 触屏长按/操作慢，给 4s（原 3s）
        onTriggered: control.armed = false
    }

    onClicked: {
        if (control.armed) {
            control.armed = false
            disarmer.stop()
            control.confirmed()
        } else {
            control.armed = true
            disarmer.restart()
        }
    }

    background: Item {
        Glow {
            id: glowFx
            anchors.fill: plate
            source: plate
            radius: 18
            spread: 0.3
            color: Theme.withAlpha(control.accent, 0.55)
            transparentBorder: true
            visible: control.armed

            SequentialAnimation on opacity {
                loops: Animation.Infinite
                running: control.armed
                NumberAnimation { from: 1.0; to: 0.45; duration: 700; easing.type: Easing.InOutSine }
                NumberAnimation { from: 0.45; to: 1.0; duration: 700; easing.type: Easing.InOutSine }
            }
        }

        Rectangle {
            id: plate
            anchors.fill: parent
            radius: height / 2

            gradient: Gradient {
                GradientStop {
                    position: 0.0
                    color: control.armed ? Theme.accentSoft
                                         : Theme.withAlpha(control.accent, 0.15)
                }
                GradientStop {
                    position: 1.0
                    color: control.armed ? control.accent
                                         : Theme.withAlpha(control.accent, 0.06)
                }
            }

            border.width: control.armed ? 0 : 1.5
            border.color: Theme.withAlpha(control.accent, control.armed ? 0.4 : 0.60)
            Behavior on border.color { ColorAnimation { duration: Theme.msColor } }

            // 触屏：按下直接缩放 0.94，无 hover
            scale: control.down ? 0.94 : 1.0
            Behavior on scale { NumberAnimation { duration: Theme.msPress; easing.type: Easing.OutQuad } }
        }
    }

    contentItem: Text {
        text: control.text
        color: control.armed ? "#0A1405" : control.accent
        font.pixelSize: 19
        font.bold: true
        font.letterSpacing: 1
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        Behavior on color { ColorAnimation { duration: Theme.msColor } }
    }
}
