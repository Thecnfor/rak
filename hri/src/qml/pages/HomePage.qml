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
            onTaskClicked: (index) => app.selectTask(index)
        }

        TaskCockpit {
            Layout.fillWidth: true
            Layout.fillHeight: true
            taskIndex: app.selectedTask
            taskName: app.tasks.length ? app.tasks[app.selectedTask].name : ""
            taskDescription: app.tasks.length ? app.tasks[app.selectedTask].description : ""
            taskSpeed: app.tasks.length ? app.tasks[app.selectedTask].speed : 0
            running: app.running
            onStartFrom: (index) => app.startFrom(index)
            onRunSingle: (index) => app.runSingle(index)
            onStopRequested: () => app.stop()
            onSpeedCommitted: (index, value) => app.setTaskSpeed(index, value)
        }
    }
}
