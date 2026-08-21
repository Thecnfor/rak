import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import com.hri.app

// 聚焦舱：触摸屏更大启动按钮 64 高 + 间距更宽 + 可滚动 + 可展开参数调节
Item {
    id: cockpit

    property int taskIndex: 0
    property string taskName: ""
    property string taskDescription: ""
    property real taskSpeed: 1.5
    property bool running: false
    property int selectedCount: 0
    property color accent: Theme.accent
    property var config: ({})          // 当前任务的参数配置 (lane PID / 触发参数)
    signal startFrom(int index)
    signal runSingle(int index)
    signal stopRequested()
    signal resetRequested()
    signal speedCommitted(int index, real value)
    signal configEdited(var newConfig)

    GlassPanel {
        anchors.fill: parent
        radius: Theme.radiusPanel

        Flickable {
            id: flick
            anchors.fill: parent
            contentWidth: width
            contentHeight: contentCol.implicitHeight
            clip: true
            boundsBehavior: Flickable.StopAtBounds
            ScrollBar.vertical: ScrollBar {
                policy: ScrollBar.AsNeeded
                width: 6
                parent: flick
                anchors.right: flick.right
                anchors.margins: 2
            }

            ColumnLayout {
                id: contentCol
                width: flick.width
                anchors.margins: 22
                spacing: 12

                Row {
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 14

                    Text {
                        text: (cockpit.taskIndex + 1) < 10 ? "0" + (cockpit.taskIndex + 1)
                                                           : (cockpit.taskIndex + 1)
                        font.pixelSize: 30
                        font.bold: true
                        color: Theme.accentSoft
                    }

                    Text {
                        text: "·"; font.pixelSize: 30; font.bold: true; color: Theme.textTertiary
                    }

                    Text {
                        text: cockpit.taskName
                        font.pixelSize: 30
                        font.bold: true
                        color: Theme.textPrimary
                    }
                }

                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: cockpit.taskDescription
                    font.pixelSize: 14
                    color: Theme.textSecondary
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.leftMargin: 60; Layout.rightMargin: 60
                    Layout.topMargin: 4
                    height: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.5; color: Theme.hairlineStrong }
                        GradientStop { position: 1.0; color: "transparent" }
                    }
                }

                SpeedControl {
                    Layout.alignment: Qt.AlignHCenter
                    value: cockpit.taskSpeed
                    accent: cockpit.accent
                    onValueCommitted: (value) => cockpit.speedCommitted(cockpit.taskIndex, value)
                }

                RowLayout {
                    Layout.alignment: Qt.AlignHCenter
                    Layout.topMargin: 10
                    spacing: 22

                    StartButton {
                        Layout.preferredWidth: 280
                        Layout.preferredHeight: 64
                        label: "▶ 开始运行（已选 " + cockpit.selectedCount + " 个）"
                        accent: cockpit.accent
                        onConfirmed: cockpit.startFrom(cockpit.taskIndex)
                    }

                    StartButton {
                        Layout.preferredWidth: 200
                        Layout.preferredHeight: 64
                        label: "⇥ 仅跑本任务"
                        accent: Theme.cyan
                        onConfirmed: cockpit.runSingle(cockpit.taskIndex)
                    }

                    Button {
                        id: stopBtn
                        visible: cockpit.running
                        Layout.preferredWidth: 150
                        Layout.preferredHeight: 64
                        text: "■ 停止"
                        hoverEnabled: false
                        onClicked: cockpit.stopRequested()

                        background: Rectangle {
                            radius: height / 2
                            color: stopBtn.down ? Theme.withAlpha(Theme.danger, 0.34)
                                                : Theme.withAlpha(Theme.danger, 0.14)
                            Behavior on color { ColorAnimation { duration: Theme.msColor } }
                            border.width: 1.5
                            border.color: Theme.withAlpha(Theme.danger, 0.7)
                            scale: stopBtn.down ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: Theme.msPress; easing.type: Easing.OutQuad } }
                        }
                        contentItem: Text {
                            text: stopBtn.text
                            font.pixelSize: 19
                            font.bold: true
                            color: Theme.dangerSoft
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }

                    // 重置进度（清空已完成，重新开始）
                    Button {
                        id: resetBtn
                        visible: !cockpit.running
                        Layout.preferredWidth: 150
                        Layout.preferredHeight: 64
                        text: "↺ 重置进度"
                        hoverEnabled: false
                        onClicked: cockpit.resetRequested()

                        background: Rectangle {
                            radius: height / 2
                            color: resetBtn.down ? Theme.withAlpha(Theme.violet, 0.34)
                                                 : Theme.withAlpha(Theme.violet, 0.14)
                            Behavior on color { ColorAnimation { duration: Theme.msColor } }
                            border.width: 1.5
                            border.color: Theme.withAlpha(Theme.violet, 0.6)
                            scale: resetBtn.down ? 0.95 : 1.0
                            Behavior on scale { NumberAnimation { duration: Theme.msPress; easing.type: Easing.OutQuad } }
                        }
                        contentItem: Text {
                            text: resetBtn.text
                            font.pixelSize: 19
                            font.bold: true
                            color: Theme.violet
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }

                // ── 参数调节（可展开）──
                Rectangle {
                    Layout.fillWidth: true
                    Layout.leftMargin: 60; Layout.rightMargin: 60
                    Layout.topMargin: 6
                    height: 1
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "transparent" }
                        GradientStop { position: 0.5; color: Theme.hairlineStrong }
                        GradientStop { position: 1.0; color: "transparent" }
                    }
                }

                Button {
                    id: configToggle
                    Layout.alignment: Qt.AlignHCenter
                    Layout.preferredWidth: 300
                    Layout.preferredHeight: 56
                    text: configOpen.checked ? "▲ 收起参数调节" : "▼ 参数调节（PID / 触发）"
                    checkable: true
                    checked: false
                    hoverEnabled: false

                    background: Rectangle {
                        radius: height / 2
                        color: configToggle.down ? Theme.withAlpha(Theme.cyan, 0.22)
                                                 : Theme.withAlpha(Theme.cyan, 0.10)
                        Behavior on color { ColorAnimation { duration: Theme.msColor } }
                        border.width: 1
                        border.color: Theme.withAlpha(Theme.cyan, 0.5)
                        scale: configToggle.down ? 0.96 : 1.0
                        Behavior on scale { NumberAnimation { duration: Theme.msPress; easing.type: Easing.OutQuad } }
                    }
                    contentItem: Text {
                        text: configToggle.text
                        font.pixelSize: 17
                        font.bold: true
                        color: Theme.cyan
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }

                Item {
                    id: configOpen
                    Layout.fillWidth: true
                    Layout.topMargin: 4
                    visible: configToggle.checked
                    height: visible ? configPanel.implicitHeight : 0
                    clip: true

                    Behavior on height { NumberAnimation { duration: Theme.msMove; easing.type: Easing.OutCubic } }

                    TaskConfigPanel {
                        id: configPanel
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        config: cockpit.config
                        accent: Theme.cyan
                        onConfigEdited: (cfg) => cockpit.configEdited(cfg)
                    }
                }
            }
        }
    }
}
