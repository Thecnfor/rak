#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""完整比赛流程入口：一键编排（comp_mode）。

按键语义（由 Orchestrator 统一接管）:
    4 = 一键启动 / 一轮结束后重来
    1 = 跳过当前任务 (不标记完成, 下次重来仍补做)
    3 = 急停退出

路段巡线特调 / 任务点触发(advance 后停) / 任务后钉底盘姿势(end_pose)
的配置在 tasks/start/trigger_configs.py 的 TASK_TRIGGER 表里逐段填。

任务结束的机械臂位姿 + 里程计重置来自 scripts/lane/lane-stop.py 的
标定流程, 已正式嵌入为编排器的 after 钩子（见 TASK_END_POSE）。
"""

from tasks.tools import create_car
from tasks.orchestrator import Orchestrator
from tasks.start.trigger_configs import TASK_ORDER


# 跳过这些任务(不跑巡线/钩子/钉姿势), 填 TASK_ORDER 里的任务名即可
# SKIP_TASKS: set = set(["seeding", "target_detection", "watering", "shooting", "harvesting", "sorting", "ordering"])
SKIP_TASKS: set = set()

# 每个任务结束后的机械臂位姿 (x, y, arm, hand) -- 手动调
# 注意: x 合法范围 -0.315~0(m), y 合法范围 -0.2~0(m); 单位是米, 都是负方向!
TASK_END_POSE = {
    "seeding": (-0.1, 0, "LEFT", "UP"),
    "target_detection": (-0.3, 0, "RIGHT", "UP"),
    "watering": (-0.3, 0, "LEFT", "UP"),
    "shooting": (0, 0, "LEFT", "DOWN"),
    "harvesting": (-0.0, 0, "LEFT", "UP"),
    "sorting": (-0.3, 0, "RIGHT", "UP"),
    "ordering": (-0, 0, "LEFT", "UP"),
    "delivery": (-0, 0, "LEFT", "UP"),
}


def _pin_arm_and_reset(car, task_name):
    """after 钩子: 任务结束钉机械臂位姿 + 重置里程计 (lane-stop 标定流程)."""
    pose = TASK_END_POSE.get(task_name)
    if pose:
        x, y, arm, hand = pose
        print(f"[{task_name}] 钉机械臂位姿: x={x} y={y} arm={arm} hand={hand}")
        car.arm.set_arm_pose(x, y, arm, hand)
    # 任务结束重置里程计 (覆盖 sorting 默认清零钩子, 统一每任务清零;
    # 触发距离都是相对本次巡线起点的, 不受影响)
    car.reset_position()
    car.get_odometry(True)
    car.get_distance(True)
    print(f"[{task_name}] 里程计已重置")


def main():
    car = create_car(
        reset=True, comp_mode=True
    )  # 初始化(含机械臂与里程计复位) + 比赛模式按键接管
    orch = Orchestrator(car)
    orch.skip = set(SKIP_TASKS)  # 静态跳过: 整个流程不跑这些任务

    # 每个任务结束: 钉机械臂位姿 + 重置里程计
    for task_name in TASK_ORDER:
        orch.set_after_hook(task_name, _pin_arm_and_reset)

    try:
        # 一键比赛流程: 等按键 4 启动, 触发后自动 run(car), 一轮结束可重来。
        # 结果链: target_detection -> shooting(animal_list),
        #         ordering -> delivery(order_list); 上游被跳过时回落任务默认值
        orch.run_all(
            auto_run_task=True,
            wait_start=True,
            allow_restart=True,
            task_kwargs={
                "shooting": {
                    "animal_list": lambda results: results.get("target_detection"),
                },
                "delivery": {
                    "order_list": lambda results: results.get("ordering"),
                },
            },
        )
    finally:
        car.stop()
        car.close()


if __name__ == "__main__":
    main()
