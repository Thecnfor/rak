pragma Singleton
import QtQuick

// 全局设计令牌: 玻璃拟态主题的配色 / 圆角 / 尺寸 / 工具函数唯一来源
// 触摸屏增强：所有交互元素最小 48dp 热区、间距 ≥8dp，便于粗手指点击
QtObject {
    // ── 基底 ──────────────────────────────────────────
    readonly property color bgBase: "#060A12"
    readonly property color bgTop:  "#0C1828"

    // ── 品牌 / 语义色 ──────────────────────────────────
    readonly property color accent:     "#76B900"
    readonly property color accentSoft: "#A7E84B"
    readonly property color cyan:       "#4FC3F7"
    readonly property color violet:     "#8B7CFF"
    readonly property color danger:     "#E5484D"
    readonly property color dangerSoft: "#FF8A8E"
    readonly property color success:    "#34D399"

    // ── 文字 ──────────────────────────────────────────
    readonly property color textPrimary:   "#F2F6FC"
    readonly property color textSecondary: "#A9B7CC"
    readonly property color textTertiary:  "#5F6E86"

    // ── 玻璃表面 ──────────────────────────────────────
    readonly property color glass:           Qt.rgba(1, 1, 1, 0.050)
    readonly property color glassRaised:     Qt.rgba(1, 1, 1, 0.085)
    readonly property color glassPressed:    Qt.rgba(1, 1, 1, 0.140)
    readonly property color hairline:        Qt.rgba(1, 1, 1, 0.09)
    readonly property color hairlineStrong:  Qt.rgba(1, 1, 1, 0.16)
    readonly property color shadow:          Qt.rgba(0, 0, 0, 0.50)

    // ── 圆角 / 尺寸（触摸屏友好：按钮≥48，卡片≥88，底部导航≥80）───────────
    readonly property int radiusPanel: 20
    readonly property int radiusCard:  14
    readonly property int radiusChip:  12

    // 触摸最小热区 (Material 设计规范 ≥ 48dp)，本应用作为最小值下限
    readonly property int touchMinSize: 56
    readonly property int touchPadSize: 48
    readonly property int touchNavMinHeight: 80

    // 间距
    readonly property int gapXS: 4
    readonly property int gapSM: 8
    readonly property int gapMD: 14
    readonly property int gapLG: 20

    // 反馈动效时长 (触屏更短：100~180ms，避免感知迟缓)
    readonly property int msPress: 90
    readonly property int msColor: 140
    readonly property int msMove: 260

    // 工具: 给颜色换透明度
    function withAlpha(c, a) {
        return Qt.rgba(c.r, c.g, c.b, a)
    }

    // 工具: Canvas 上下文 "rgba(...)" 字符串
    function css(c, a) {
        return "rgba(" + Math.round(c.r * 255) + "," + Math.round(c.g * 255)
               + "," + Math.round(c.b * 255) + "," + a + ")"
    }

    // 工具: 保证至少 48x48 触摸热区（把小控件包在 Item 里，用 MouseArea 扩展）
    function padHit(w, h) {
        return {
            w: Math.max(w, Theme.touchPadSize),
            h: Math.max(h, Theme.touchPadSize),
            dx: Math.max(0, Theme.touchPadSize - w) / 2,
            dy: Math.max(0, Theme.touchPadSize - h) / 2,
        }
    }
}
