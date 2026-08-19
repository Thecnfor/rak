import QtQuick
import Qt5Compat.GraphicalEffects
import com.hri.app

// 底部导航：触摸屏加大高度 104 (原 88)，页签 72x72 热区，整块可点
Item {
    id: nav
    height: 104

    property var tabs: [
        { icon: "home",  label: "主控", index: 0 },
        { icon: "gauge", label: "状态", index: 1 },
        { icon: "gear",  label: "设置", index: 2 }
    ]

    GlassPanel {
        id: bar
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.topMargin: 4
        anchors.leftMargin: 20
        anchors.rightMargin: 20
        anchors.bottomMargin: 10
        radius: 999

        Rectangle {
            id: indicator
            x: 6 + (bar.width / nav.tabs.length) * app.currentPage
            y: 6
            width: bar.width / nav.tabs.length - 12
            height: bar.height - 12
            radius: height / 2

            Behavior on x { NumberAnimation { duration: Theme.msMove; easing.type: Easing.OutCubic } }

            gradient: Gradient {
                GradientStop { position: 0.0; color: Theme.withAlpha(Theme.accent, 0.26) }
                GradientStop { position: 1.0; color: Theme.withAlpha(Theme.accent, 0.10) }
            }
            border.width: 1
            border.color: Theme.withAlpha(Theme.accent, 0.55)
        }

        Row {
            anchors.fill: parent

            Repeater {
                model: nav.tabs

                Item {
                    id: tab
                    width: bar.width / nav.tabs.length
                    height: bar.height

                    property bool active: app.currentPage === modelData.index

                    // 整块可点热区 72x72+
                    MouseArea {
                        id: mouseArea
                        anchors.fill: parent
                        hoverEnabled: false           // 触摸屏无 hover
                        preventStealing: true
                        onClicked: app.setPage(modelData.index)
                    }

                    Column {
                        anchors.centerIn: parent
                        spacing: 4

                        GlyphIcon {
                            anchors.horizontalCenter: parent.horizontalCenter
                            width: 28; height: 28
                            glyph: modelData.icon
                            stroke: tab.active ? Theme.accentSoft : Theme.textTertiary
                            Behavior on stroke { ColorAnimation { duration: Theme.msColor } }
                        }

                        Text {
                            anchors.horizontalCenter: parent.horizontalCenter
                            text: modelData.label
                            font.pixelSize: 13
                            font.bold: tab.active
                            color: tab.active ? Theme.accentSoft : Theme.textTertiary
                            Behavior on color { ColorAnimation { duration: Theme.msColor } }
                        }
                    }
                }
            }
        }
    }
}
