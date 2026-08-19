import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Qt5Compat.GraphicalEffects
import com.hri.app

// HRI 根窗口：触摸屏独占全屏（1024x600 为目标分辨率）
// - 无边框 / 无标题栏
// - 无鼠标光标（隐藏，用触摸）
// - 触摸滚动：Flickable 包裹可滚动区域
// - 防止误触：底部导航/启动按钮等加大热区
// - 虚拟键盘：需要时弹出 QtQuick.VirtualKeyboard（系统需带模块）
ApplicationWindow {
    id: root

    // 全屏模式（__hri_fullscreen / __hri_kiosk 由 main.cpp 注入）
    // 若开发机 windowed 模式：1024x600 居中预览；否则交给 main.cpp 的 fullscreen 设置
    visible: true
    visibility: (__hri_fullscreen || __hri_kiosk) ? Window.FullScreen : Window.Windowed

    // Kiosk：完全去掉系统装饰（Alt+F4/最大化最小化不可点），全屏锁界面
    flags: (__hri_kiosk
            ? (Window.FramelessWindowHint | Window.WindowStaysOnTopHint)
            : (__hri_fullscreen ? Window.FramelessWindowHint : 0))

    color: Theme.bgBase

    // ─────────────────────────────────────────────────────
    // 触摸滚动容器（整个 UI 放进去，超屏时可滚）
    // 目标 1024x600 一般不需要滚动，但可兼容大屏/小屏
    // ─────────────────────────────────────────────────────
    Flickable {
        id: shell
        anchors.fill: parent
        contentWidth: Math.max(parent.width, 1024 * __hri_scale)
        contentHeight: Math.max(parent.height, 600 * __hri_scale)
        interactive: contentWidth > width || contentHeight > height
        boundsBehavior: Flickable.StopAtBounds
        pixelAligned: true

        // 触摸反馈：按下时轻微缩放的全局效果（由各控件各自实现，这里不全局）
        // 设计稿基准 1024x600；__hri_scale 默认 1，可用 --scale=110 放大触控
        Item {
            id: content
            width: 1024; height: 600

            transform: Scale {
                origin.x: 0; origin.y: 0
                xScale: __hri_scale; yScale: __hri_scale
            }

            // ── 背景: 深空纵向渐变 ─────────────────────────────
            Rectangle {
                anchors.fill: parent
                gradient: Gradient {
                    GradientStop { position: 0.0; color: Theme.bgTop }
                    GradientStop { position: 1.0; color: Theme.bgBase }
                }
            }

            // 极光光晕
            component AuroraOrb: Item {
                id: orb
                property color tint
                property real strength: 1.0
                Repeater {
                    model: [1.0, 0.78, 0.58, 0.40, 0.26]
                    Rectangle {
                        required property real modelData
                        property real k: modelData
                        width: 620 * k; height: 620 * k
                        anchors.centerIn: parent
                        radius: width / 2
                        color: orb.tint
                        opacity: 0.05 * orb.strength * (1.2 - k)
                    }
                }
            }
            AuroraOrb { x: parent.width - 140; y: -40;  tint: Theme.accent; strength: 1.5 }
            AuroraOrb { x: -20;           y: parent.height - 100; tint: Theme.cyan;   strength: 1.1 }
            AuroraOrb { x: parent.width - 100; y: parent.height - 60; tint: Theme.violet; strength: 0.8 }

            // ── 顶部玻璃标题栏（含返回/退出在 Kiosk 模式下不显示，防止误操作退出）────────
            Rectangle {
                id: topBar
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: 64        // 触屏加大：64 (原 52)
                color: Theme.withAlpha(Theme.bgBase, 0.55)

                Rectangle {
                    anchors.left: parent.left; anchors.right: parent.right
                    anchors.bottom: parent.bottom; height: 1
                    color: Theme.hairline
                }

                RowLayout {
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.leftMargin: 18
                    anchors.rightMargin: 18
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 14

                    Rectangle {
                        width: 40; height: 40; radius: 12
                        gradient: Gradient {
                            GradientStop { position: 0.0; color: Theme.accentSoft }
                            GradientStop { position: 1.0; color: Theme.accent }
                        }
                        Text {
                            anchors.centerIn: parent
                            text: "H"; font.pixelSize: 20; font.bold: true; color: "#0A1405"
                        }
                        // 触摸扩展热区
                        MouseArea {
                            anchors.fill: parent
                            preventStealing: true
                            onPressAndHold: {
                                // 长按徽标 2s：Kiosk 模式下可暴露紧急退出（长按 3 次后）
                                guard.tripleHold()
                            }
                        }
                    }

                    Text {
                        text: app.pageTitle
                        color: Theme.textPrimary
                        font.pixelSize: 22
                        font.bold: true
                        font.letterSpacing: 1
                    }

                    Item { Layout.fillWidth: true }

                    // 全局状态胶囊行（触屏加大字体/间距）
                    Row {
                        spacing: 12

                        // 运行态胶囊
                        Rectangle {
                            height: 38; width: statusRow.width + 28; radius: 19
                            color: app.running ? Theme.withAlpha(Theme.accent, 0.10)
                                               : Theme.withAlpha(Theme.bgTop, 0.45)
                            border.width: 1
                            border.color: app.running ? Theme.withAlpha(Theme.accent, 0.45)
                                                      : Theme.hairline
                            Row {
                                id: statusRow
                                anchors.centerIn: parent
                                spacing: 10
                                Item {
                                    width: 10; height: 10
                                    Rectangle {
                                        id: runDot
                                        anchors.fill: parent; radius: 5
                                        color: app.running ? Theme.accentSoft : Theme.textTertiary
                                    }
                                    Glow {
                                        anchors.fill: runDot; source: runDot
                                        radius: 7; color: Theme.accent; spread: 0.35
                                        visible: app.running
                                        SequentialAnimation on opacity {
                                            loops: Animation.Infinite; running: app.running
                                            NumberAnimation { from: 1.0; to: 0.35; duration: 900; easing.type: Easing.InOutSine }
                                            NumberAnimation { from: 0.35; to: 1.0; duration: 900; easing.type: Easing.InOutSine }
                                        }
                                    }
                                }
                                Text {
                                    text: app.running ? "运行中" : "已停止"
                                    color: app.running ? Theme.accentSoft : Theme.textSecondary
                                    font.pixelSize: 14; font.bold: app.running
                                }
                            }
                        }

                        // 进度胶囊
                        Rectangle {
                            height: 38; width: progressText.width + 28; radius: 19
                            color: Theme.withAlpha(Theme.bgTop, 0.45)
                            border.width: 1; border.color: Theme.hairline
                            Text {
                                id: progressText
                                anchors.centerIn: parent
                                text: "任务 " + (app.currentTask >= 0 ? (app.currentTask + 1) : "—")
                                      + " / " + app.tasks.length
                                color: Theme.textSecondary; font.pixelSize: 14
                            }
                        }

                        // 速度胶囊
                        Rectangle {
                            height: 38; width: speedText.width + 28; radius: 19
                            color: Theme.withAlpha(Theme.accent, 0.10)
                            border.width: 1
                            border.color: Theme.withAlpha(Theme.accent, 0.40)
                            Text {
                                id: speedText
                                anchors.centerIn: parent
                                text: app.currentSpeed.toFixed(1) + " m/s"
                                color: Theme.accentSoft; font.pixelSize: 15; font.bold: true
                            }
                        }
                    }
                }
            }

            // ── 页面栈 ───────────────────────────────────────
            StackLayout {
                id: pageStack
                anchors.top: topBar.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.bottom: navBar.top

                currentIndex: app.currentPage

                HomePage {}
                StatusPage {}
                SettingsPage {}
            }

            // ── 底部导航（悬浮玻璃圆盘）─────────────────────
            BottomNav {
                id: navBar
                anchors.left: parent.left; anchors.right: parent.right
                anchors.bottom: parent.bottom
            }

            // ── 紧急退出守卫（Kiosk：长按徽标 3 次确认）─────
            Item {
                id: guard
                property int _holdCount: 0
                Timer {
                    id: holdReset
                    interval: 4000
                    onTriggered: guard._holdCount = 0
                }
                function tripleHold() {
                    guard._holdCount += 1
                    holdReset.restart()
                    if (guard._holdCount >= 3) {
                        guard._holdCount = 0
                        // 仅在 kiosk 模式下生效，避免 windowed 误触
                        if (__hri_kiosk) {
                            notice.show("长按 3 次，5 秒后自动重启主控台")
                            Qt.quit()
                        } else if (__hri_fullscreen) {
                            notice.show("已请求退出全屏")
                            root.visibility = Window.Windowed
                            root.flags = 0
                        }
                    }
                }
            }

            // ── 用户提示条（底部居中玻璃胶囊）──────────────
            Item {
                id: notice
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: navBar.top
                anchors.bottomMargin: 14
                width: noticePlate.width
                height: noticePlate.visible ? noticePlate.height : 0
                clip: true

                Rectangle {
                    id: noticePlate
                    width: 420
                    height: 44
                    radius: 22
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.bottom: parent.bottom
                    color: Theme.withAlpha("#FFFFFF", 0.08)
                    border.color: Theme.hairlineStrong; border.width: 1
                    visible: opacity > 0.01
                    opacity: 0
                    Behavior on opacity { NumberAnimation { duration: 180 } }

                    Text {
                        id: noticeText
                        anchors.centerIn: parent
                        color: Theme.textPrimary
                        font.pixelSize: 14
                    }

                    Timer {
                        id: noticeTimer
                        interval: 2600
                        onTriggered: noticePlate.opacity = 0
                    }
                }

                function show(msg) {
                    noticeText.text = msg
                    noticePlate.opacity = 1
                    noticeTimer.restart()
                }

                Connections {
                    target: app
                    function onNotice(message) { notice.show(message) }
                }
            }

            // ── 屏幕常亮 / 屏保抑制占位（系统层需要外部配合）─
            Timer {
                interval: 30000; running: true; repeat: true
                onTriggered: {
                    // 虚拟按键事件：每 30s 发一次 Move 到 (1,1)，抑制 X11 黑屏
                    // 真机要开 "xset s off" + "xset -dpms"，这里仅兜底
                }
            }
        }
    }
}
