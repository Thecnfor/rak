#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""完整比赛流程入口：一键编排（comp_mode）。

按键语义（由 Orchestrator 统一接管）:
    4 = 一键启动 / 一轮结束后重来
    1 = 跳过当前任务 (不标记完成, 下次重来仍补做)
    3 = 急停退出

路段巡线特调 / 任务点触发(advance 后停) / 任务后钉姿势(end_pose)
的配置在 tasks/start/trigger_configs.py 的 TASK_TRIGGER 表里逐段填。
"""

from tasks.tools import create_car
from tasks.orchestrator import Orchestrator
from tasks.target_detection import run


def main():
    car = create_car(reset=True, comp_mode=True)  # 初始化(含机械臂与里程计复位) + 比赛模式按键接管
    orch = Orchestrator(car)
    try:
        # 一键比赛流程: 触发后自动 run(car), 等按键 4 启动, 一轮结束可重来
        while True:
            animal_list = run(car)
            print(f"animal_list = {animal_list}")
            pass
        orch.run_all(auto_run_task=True, wait_start=True, allow_restart=True)
    finally:
        car.stop()
        car.close()


if __name__ == "__main__":
    main()
