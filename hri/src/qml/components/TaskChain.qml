import QtQuick
import com.hri.app

// 横排任务链: 玻璃任务卡 + 顺序连接线, 点选聚焦, 整体居中
// 触摸屏交互: 点击卡片 = 聚焦该任务 + 切换勾选（选中哪几个就只跑哪几个）
Item {
    id: chain

    property var tasks: []
    property int selectedTask: 0
    property int currentTask: -1
    property bool running: false
    property color accent: Theme.accent
    signal taskClicked(int index)

    height: 96

    Row {
        anchors.centerIn: parent
        spacing: 0

        // 偶数位渲染任务卡, 奇数位渲染连接线 (tasks.length*2-1 交替展开)
        Repeater {
            model: Math.max(0, chain.tasks.length * 2 - 1)

            Item {
                id: slot
                width: index % 2 === 0 ? 110 : 14
                height: chain.height

                // 任务卡
                TaskCard {
                    visible: index % 2 === 0
                    anchors.centerIn: parent
                    taskIndex: Math.floor(index / 2)
                    taskName: index % 2 === 0 ? chain.tasks[Math.floor(index / 2)].name : ""
                    status: index % 2 === 0 ? chain.tasks[Math.floor(index / 2)].status : ""
                    selected: Math.floor(index / 2) === chain.selectedTask
                    checked: index % 2 === 0 ? chain.tasks[Math.floor(index / 2)].selected : true
                    accent: chain.accent
                    onClicked: (i) => chain.taskClicked(i)
                }

                // 顺序连接线: 前一张卡完成后变为品牌绿, 表达进度流动
                Rectangle {
                    visible: index % 2 === 1
                    anchors.centerIn: parent
                    width: 14
                    height: 2
                    radius: 1
                    color: index % 2 === 1
                           && chain.tasks[Math.floor(index / 2)].status === "done"
                           ? Theme.withAlpha(chain.accent, 0.65)
                           : Theme.hairline
                    Behavior on color { ColorAnimation { duration: 250 } }
                }
            }
        }
    }
}
