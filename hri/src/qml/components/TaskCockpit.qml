import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import com.hri.app

// 聚焦舱：触摸屏更大启动按钮 64 高 + 间距更宽
Item {
    id: cockpit

    property int taskIndex: 0
    property string taskName: ""
    property string taskDescription: ""
    property real taskSpeed: 1.5
    property bool running: false
    property color accent: Theme.accent
    signal startFrom(int index)
    signal runSingle(int index)
    signal stopRequested()
    signal speedCommitted(int index, real value)

    GlassPanel {
        anchors.fill: parent
        radius: Theme.radiusPanel

        ColumnLayout {
            anchors.fill: parent
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
                    label: "▶ 从本任务开始运行"
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
            }
        }
    }
}
