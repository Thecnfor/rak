import QtQuick
import QtQuick.Layouts
import com.hri.app

// 主控台: 横排任务链 + 选中任务聚焦舱 (A+C)
Rectangle {
    color: "transparent"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 14

        TaskChain {
            Layout.fillWidth: true
            tasks: app.tasks
            selectedTask: app.selectedTask
            currentTask: app.currentTask
            running: app.running
            // 点击卡片 = 聚焦该任务 + 切换勾选（选中哪几个就只跑哪几个）
            onTaskClicked: (index) => {
                app.selectTask(index)
                app.toggleTaskSelected(index)
            }
        }

        TaskCockpit {
            Layout.fillWidth: true
            Layout.fillHeight: true
            taskIndex: app.selectedTask
            taskName: app.tasks.length ? app.tasks[app.selectedTask].name : ""
            taskDescription: app.tasks.length ? app.tasks[app.selectedTask].description : ""
            taskSpeed: app.tasks.length ? app.tasks[app.selectedTask].speed : 0
            running: app.running
            selectedCount: app.selectedCount
            config: app.taskConfig
            onStartFrom: (index) => app.startFrom(index)
            onRunSingle: (index) => app.runSingle(index)
            onStopRequested: () => app.stop()
            onResetRequested: () => app.reset()
            onSpeedCommitted: (index, value) => app.setTaskSpeed(index, value)
            onConfigEdited: (cfg) => app.setTaskConfig(app.selectedTask, cfg)
        }
    }
}
