import QtQuick
import com.hri.app

// 设置页: 玻璃面板占位 (后续放参数配置入口)
Rectangle {
    color: "transparent"

    GlassPanel {
        anchors.centerIn: parent
        width: 420
        height: 240
        radius: Theme.radiusPanel

        Column {
            anchors.centerIn: parent
            spacing: 10

            GlyphIcon {
                anchors.horizontalCenter: parent.horizontalCenter
                width: 44
                height: 44
                glyph: "gear"
                stroke: Theme.accentSoft
                pen: 3
            }

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "设置"
                font.pixelSize: 24
                font.bold: true
                color: Theme.textPrimary
            }

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "这里将放置机器人参数配置入口"
                font.pixelSize: 13
                color: Theme.textSecondary
            }
        }
    }
}
