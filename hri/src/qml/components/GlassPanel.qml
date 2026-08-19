import QtQuick
import Qt5Compat.GraphicalEffects
import com.hri.app

// 通用玻璃拟态面板: 半透渐变底 + 顶部受光棱 + 细描边 + 柔和投影
// 用法: 当普通容器用, 直接把子元素写进去 (经默认属性落入内容层, 自动裁圆角)
Item {
    id: root

    default property alias content: holder.data
    property int radius: Theme.radiusPanel
    property color fill: Theme.glass             // 玻璃底 (下端)
    property color fillTop: Theme.glassRaised    // 玻璃底 (顶端受光面)
    property color borderColor: Theme.hairline
    property bool shadow: true                   // 柔和投影 (静态内容, cached 只栅格化一次)

    // 实际圆角: 防止传入超大半径 (如胶囊) 时高光计算溢出
    readonly property int effRadius: Math.min(radius, Math.min(width, height) / 2)

    // 柔和投影: 外扩锚定给辉光留足空间, 避免被自身边界裁掉
    DropShadow {
        anchors.fill: surface
        anchors.margins: -14
        source: surface
        radius: 18
        color: Theme.shadow
        verticalOffset: 6
        transparentBorder: true
        cached: root.shadow
        visible: root.shadow
    }

    Rectangle {
        id: surface
        anchors.fill: parent
        radius: root.effRadius
        clip: true

        // 玻璃底: 顶端微亮 → 底部更透, 像被从上方照亮
        gradient: Gradient {
            GradientStop { position: 0.0; color: root.fillTop }
            GradientStop { position: 1.0; color: root.fill }
        }

        border.width: 1
        border.color: root.borderColor

        // 顶部受光棱: 中段亮、两端隐去的 1px 高光线, 玻璃拟态的关键细节
        Rectangle {
            x: root.effRadius
            width: Math.max(0, parent.width - root.effRadius * 2)
            height: 1
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.0; color: "transparent" }
                GradientStop { position: 0.5; color: Qt.rgba(1, 1, 1, 0.20) }
                GradientStop { position: 1.0; color: "transparent" }
            }
        }

        // 内容层 (clip 在 surface 上, 子元素不会画出圆角外)
        Item {
            id: holder
            anchors.fill: parent
        }
    }
}
