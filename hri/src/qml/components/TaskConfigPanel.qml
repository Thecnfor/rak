import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import com.hri.app

// 参数调节面板：显示某任务的 lane PID + 触发参数，± 步进调节
// 每次调节后把更新后的完整 config 通过 configChanged 抛给上层
Item {
    id: root

    property var config: ({})          // 后端返回的任务配置 (QVariantMap)
    property color accent: Theme.accent
    signal configEdited(var newConfig)

    // 读取 lane 子对象里的数值
    function laneVal(key, def) {
        var lane = root.config["lane"]
        if (lane && lane[key] !== undefined)
            return Number(lane[key])
        return def
    }
    // 读取顶层数值
    function topVal(key, def) {
        if (root.config[key] !== undefined)
            return Number(root.config[key])
        return def
    }
    // 更新 lane 子对象里的数值并抛出新 config
    function setLane(key, val) {
        var cfg = {}
        for (var k in root.config) cfg[k] = root.config[k]
        var lane = {}
        var oldLane = root.config["lane"]
        if (oldLane) for (var lk in oldLane) lane[lk] = oldLane[lk]
        lane[key] = val
        cfg["lane"] = lane
        root.configEdited(cfg)
    }
    // 更新顶层数值并抛出新 config
    function setTop(key, val) {
        var cfg = {}
        for (var k in root.config) cfg[k] = root.config[k]
        cfg[key] = val
        root.configEdited(cfg)
    }

    readonly property string trigType: root.config["type"] || ""

    implicitWidth: 320
    implicitHeight: col.implicitHeight

    ColumnLayout {
        id: col
        anchors.left: parent.left
        anchors.right: parent.right
        spacing: 10

        // ── 巡线 PID ──
        Text {
            text: "巡线 PID"
            font.pixelSize: 13
            font.bold: true
            color: Theme.textTertiary
            Layout.topMargin: 2
        }

        ParamStepper {
            Layout.alignment: Qt.AlignHCenter
            label: "Kp"
            value: root.laneVal("kp", 1.0)
            minValue: 0; maxValue: 10; step: 0.05; decimals: 2
            accent: root.accent
            onValueCommitted: (v) => root.setLane("kp", v)
        }
        ParamStepper {
            Layout.alignment: Qt.AlignHCenter
            label: "Kd"
            value: root.laneVal("kd", 0.0)
            minValue: 0; maxValue: 5; step: 0.05; decimals: 2
            accent: root.accent
            onValueCommitted: (v) => root.setLane("kd", v)
        }
        ParamStepper {
            Layout.alignment: Qt.AlignHCenter
            label: "死区"
            value: root.laneVal("deadzone", 0.0)
            minValue: 0; maxValue: 1; step: 0.01; decimals: 2
            accent: root.accent
            onValueCommitted: (v) => root.setLane("deadzone", v)
        }
        ParamStepper {
            Layout.alignment: Qt.AlignHCenter
            label: "前进速度"
            value: root.laneVal("v_forward", 0.3)
            minValue: 0.05; maxValue: 2.0; step: 0.05; decimals: 2
            accent: root.accent
            onValueCommitted: (v) => root.setLane("v_forward", v)
        }

        // ── 触发参数 ──
        Text {
            text: "触发参数"
            font.pixelSize: 13
            font.bold: true
            color: Theme.textTertiary
            Layout.topMargin: 6
        }

        // 里程计触发: distance
        ParamStepper {
            visible: root.trigType === "odometer"
            Layout.alignment: Qt.AlignHCenter
            label: "触发距离"
            value: root.topVal("distance", 1.0)
            minValue: 0; maxValue: 10; step: 0.05; decimals: 2
            accent: root.accent
            onValueCommitted: (v) => root.setTop("distance", v)
        }

        // 视觉触发: min_score / confirm
        ParamStepper {
            visible: root.trigType === "vision"
            Layout.alignment: Qt.AlignHCenter
            label: "最低置信度"
            value: root.topVal("min_score", 0.5)
            minValue: 0; maxValue: 1; step: 0.05; decimals: 2
            accent: root.accent
            onValueCommitted: (v) => root.setTop("min_score", v)
        }
        ParamStepper {
            visible: root.trigType === "vision"
            Layout.alignment: Qt.AlignHCenter
            label: "确认帧数"
            value: root.topVal("confirm", 1)
            minValue: 1; maxValue: 10; step: 1; decimals: 0
            accent: root.accent
            onValueCommitted: (v) => root.setTop("confirm", v)
        }

        // 公共: max_run / start_dist / time_out
        ParamStepper {
            Layout.alignment: Qt.AlignHCenter
            label: "兜底距离"
            value: root.topVal("max_run", 0.0)
            minValue: 0; maxValue: 10; step: 0.1; decimals: 1
            accent: root.accent
            onValueCommitted: (v) => root.setTop("max_run", v)
        }
        ParamStepper {
            Layout.alignment: Qt.AlignHCenter
            label: "起始距离"
            value: root.topVal("start_dist", 0.0)
            minValue: 0; maxValue: 10; step: 0.1; decimals: 1
            accent: root.accent
            onValueCommitted: (v) => root.setTop("start_dist", v)
        }
        ParamStepper {
            Layout.alignment: Qt.AlignHCenter
            label: "超时(秒)"
            value: root.topVal("time_out", 0.0)
            minValue: 0; maxValue: 600; step: 10; decimals: 0
            accent: root.accent
            onValueCommitted: (v) => root.setTop("time_out", v)
        }
    }
}
