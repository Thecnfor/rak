import QtQuick
import Qt5Compat.GraphicalEffects
import com.hri.app

// 任务链中的单张任务卡：触摸屏加大 (原 110x96 → 124x104)，整块可点，按下 0.94 缩放
Item {
    id: card

    property int taskIndex: 0
    property string taskName: ""
    property string status: "pending"
    property bool selected: false      // 聚焦选中（聚焦舱显示哪个）
    property bool checked: true        // 勾选参与执行（触摸屏选中哪几个就只跑哪几个）
    property color accent: Theme.accent
    signal clicked(int index)

    width: 124
    height: 104

    readonly property color statusColor: {
        if (status === "running") return accent
        if (status === "done")    return Theme.success
        if (status === "failed")  return Theme.danger
        return Theme.textTertiary
    }

    readonly property color fillColor: {
        if (selected)            return Theme.withAlpha(accent, 0.13)
        if (status === "running") return Theme.withAlpha(accent, 0.08)
        if (status === "done")    return Theme.withAlpha(Theme.success, 0.06)
        if (status === "failed")  return Theme.withAlpha(Theme.danger, 0.08)
        return Theme.glass
    }

    // 未勾选参与执行：整体淡化，提示"本轮不跑"
    opacity: card.checked ? 1.0 : 0.45
    Behavior on opacity { NumberAnimation { duration: Theme.msColor } }

    DropShadow {
        anchors.fill: face
        anchors.margins: -6
        source: face
        radius: 14
        color: Theme.withAlpha(card.accent, 0.35)
        visible: card.selected
    }

    Rectangle {
        id: face
        anchors.fill: parent
        radius: Theme.radiusCard
        color: card.fillColor
        border.width: card.selected ? 1.5 : 1
        border.color: card.selected ? Theme.withAlpha(card.accent, 0.80)
                                    : (card.status === "running" ? Theme.withAlpha(card.accent, 0.45)
                                                                 : Theme.hairline)

        // 触屏反馈：按下 0.96，无 hover
        scale: mouseArea.pressed ? 0.96 : 1.0
        Behavior on scale { NumberAnimation { duration: Theme.msPress; easing.type: Easing.OutQuad } }
        Behavior on color { ColorAnimation { duration: Theme.msColor } }
        Behavior on border.color { ColorAnimation { duration: Theme.msColor } }

        Rectangle {
            x: Theme.radiusCard
            width: parent.width - Theme.radiusCard * 2
            height: 1
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.0; color: "transparent" }
                GradientStop { position: 0.5; color: Qt.rgba(1, 1, 1, 0.18) }
                GradientStop { position: 1.0; color: "transparent" }
            }
        }

        Column {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 6

            Row {
                width: parent.width
                spacing: 8

                Text {
                    text: (card.taskIndex + 1) < 10 ? "0" + (card.taskIndex + 1)
                                                    : (card.taskIndex + 1)
                    font.pixelSize: 14
                    font.bold: true
                    font.letterSpacing: 1
                    color: card.statusColor
                }

                Item {
                    width: 12; height: 12
                    anchors.verticalCenter: parent.verticalCenter

                    Rectangle {
                        id: statusDot
                        anchors.fill: parent; radius: 6
                        color: card.statusColor
                    }

                    Glow {
                        anchors.fill: statusDot; source: statusDot
                        radius: 8; color: card.statusColor; spread: 0.35
                        visible: card.status === "running" || card.selected

                        SequentialAnimation on opacity {
                            loops: Animation.Infinite
                            running: card.status === "running"
                            NumberAnimation { from: 1.0; to: 0.35; duration: 800; easing.type: Easing.InOutSine }
                            NumberAnimation { from: 0.35; to: 1.0; duration: 800; easing.type: Easing.InOutSine }
                        }
                    }
                }
            }

            Text {
                width: parent.width
                text: card.taskName
                font.pixelSize: 16
                color: Theme.textPrimary
                elide: Text.ElideRight
            }

            Text {
                text: {
                    if (card.status === "running") return "运行中"
                    if (card.status === "done")    return "已完成"
                    if (card.status === "failed")  return "中断"
                    return "待运行"
                }
                font.pixelSize: 12
                color: card.statusColor
            }
        }

        // 勾选角标：右上角对勾（参与本轮执行）
        Rectangle {
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.margins: 6
            width: 20; height: 20; radius: 10
            color: card.checked ? Theme.withAlpha(card.accent, 0.90) : Theme.withAlpha(Theme.bgTop, 0.55)
            border.width: 1
            border.color: card.checked ? Theme.withAlpha(card.accent, 0.6) : Theme.hairline
            Text {
                anchors.centerIn: parent
                text: card.checked ? "✓" : ""
                font.pixelSize: 13; font.bold: true
                color: "#0A1405"
            }
        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: false
        preventStealing: true
        onClicked: card.clicked(card.taskIndex)
    }
}
