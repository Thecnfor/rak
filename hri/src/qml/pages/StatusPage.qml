import QtQuick
import com.hri.app

// 状态页: 玻璃面板占位 (后续扩展为传感器/运行数据看板)
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
                glyph: "gauge"
                stroke: Theme.accentSoft
                pen: 3
            }

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "状态"
                font.pixelSize: 24
                font.bold: true
                color: Theme.textPrimary
            }

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "这里将展示机器人运行状态与传感器数据"
                font.pixelSize: 13
                color: Theme.textSecondary
            }

            // 后端连接实况
            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 8

                Rectangle {
                    width: 8
                    height: 8
                    radius: 4
                    anchors.verticalCenter: parent.verticalCenter
                    color: app.backendOnline ? Theme.accent : Theme.danger
                }

                Text {
                    anchors.verticalCenter: parent.verticalCenter
                    text: (app.backendOnline ? "后端已连接" : "后端离线") + " · " + app.backendAddress
                    font.pixelSize: 12
                    color: Theme.textTertiary
                }
            }
        }
    }
}
