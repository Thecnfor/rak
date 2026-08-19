import QtQuick

// 自绘矢量小图标 (Canvas): 替代字体 unicode 符号, 跨平台字形稳定、描边风格统一
// glyph: "home" (主控) | "gauge" (状态) | "gear" (设置)
Canvas {
    id: icon

    property string glyph: "home"
    property color stroke: "#A9B7CC"
    property real pen: 2.0

    antialiasing: true
    onStrokeChanged: requestPaint()
    onGlyphChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()

    onPaint: {
        var ctx = getContext("2d")
        ctx.clearRect(0, 0, width, height)   // 无 reset() 兼容问题, 手动清画布
        ctx.strokeStyle = stroke.toString()
        ctx.lineWidth = pen
        ctx.lineCap = "round"
        ctx.lineJoin = "round"

        var w = width, h = height, cx = w / 2, cy = h / 2

        if (glyph === "home") {
            // 屋顶
            ctx.beginPath()
            ctx.moveTo(w * 0.14, h * 0.50)
            ctx.lineTo(w * 0.50, h * 0.16)
            ctx.lineTo(w * 0.86, h * 0.50)
            ctx.stroke()
            // 房身
            ctx.beginPath()
            ctx.moveTo(w * 0.26, h * 0.46)
            ctx.lineTo(w * 0.26, h * 0.84)
            ctx.lineTo(w * 0.74, h * 0.84)
            ctx.lineTo(w * 0.74, h * 0.46)
            ctx.stroke()
        } else if (glyph === "gauge") {
            // 仪表弧
            ctx.beginPath()
            ctx.arc(cx, h * 0.62, w * 0.34, Math.PI * 0.80, Math.PI * 0.20, false)
            ctx.stroke()
            // 指针 (指向高速区)
            ctx.beginPath()
            ctx.moveTo(cx, h * 0.62)
            ctx.lineTo(w * 0.70, h * 0.34)
            ctx.stroke()
            // 轴心点
            ctx.beginPath()
            ctx.arc(cx, h * 0.62, w * 0.07, 0, Math.PI * 2)
            ctx.fillStyle = stroke.toString()
            ctx.fill()
        } else { // gear: 中心环 + 八根轮齿
            var r0 = w * 0.20, r1 = w * 0.44
            ctx.beginPath()
            ctx.arc(cx, cy, r0, 0, Math.PI * 2)
            ctx.stroke()
            for (var i = 0; i < 8; i++) {
                var a = Math.PI / 8 + i * Math.PI / 4   // 错开半格, 避开 12 点正上方呆板对称
                ctx.beginPath()
                ctx.moveTo(cx + Math.cos(a) * (r0 + w * 0.035), cy + Math.sin(a) * (r0 + w * 0.035))
                ctx.lineTo(cx + Math.cos(a) * r1, cy + Math.sin(a) * r1)
                ctx.stroke()
            }
        }
    }
}
