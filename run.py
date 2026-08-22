#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""完整比赛流程入口：一键编排（comp_mode）。

按键语义（由 Orchestrator 统一接管，全部点按）:
    等待态:
        4 = 全量开始（自动清空上一轮进度，从第 1 个任务跑完整轮）
        1 = 跳过第 1 个任务开始（只跳过任务 run 逻辑，仍正常巡线到任务点停，
            后续任务正常执行）
        2 = 跳过第 1、2 个任务开始（暂保留原逻辑，待他人修改）
        3 = 立即重新初始化（蜂鸣+机械臂复位+里程计清零+清空进度，
            回到开机初始状态，像全新的一样）
    运行中:
        1 = 跳过当前任务 (不标记完成, 下次重来仍补做)
        3 = 停止本轮，回到等待态

路段巡线特调 / 任务点触发(advance 后停) / 任务后钉底盘姿势(end_pose)
的配置在 tasks/start/trigger_configs.py 的 TASK_TRIGGER 表里逐段填。

任务结束的机械臂位姿 + 里程计重置来自 scripts/lane/lane-stop.py 的
标定流程, 已正式嵌入为编排器的 after 钩子（见 TASK_END_POSE）。
"""

from tasks.tools import create_car
from tasks.orchestrator import Orchestrator
from tasks.start.trigger_configs import TASK_ORDER

import math
import os
import sys
import time


# 跳过这些任务(不跑巡线/钩子/钉姿势), 填 TASK_ORDER 里的任务名即可
# SKIP_TASKS: set = set(["seeding", "target_detection", "watering", "shooting", "harvesting", "sorting", "ordering"])
SKIP_TASKS: set = set(["harvesting", "sorting"])

# 任务结束后向左转 (逆时针) 的任务 — 起步巡线前先调整朝向
# move_for 第三分量 = 角度偏移, 正向逆时针; 角度单位弧度
# seeding 转 15° (π/12), target_detection/watering 转 30° (π/6)
_TURN_LEFT_RAD = {
    "seeding": -(math.pi / 10),
    "target_detection": math.pi / 6,
    "watering": math.pi / 6,
}

# 每个任务结束后的机械臂位姿 (x, y, arm, hand) -- 手动调
# 注意: x 合法范围 -0.315~0(m), y 合法范围 -0.2~0(m); 单位是米, 都是负方向!
TASK_END_POSE = {
    "seeding": (-0.1, -0.05, "LEFT", "UP"),
    "target_detection": (-0.2, -0.02, "RIGHT", "UP"),  # 对齐 watering.DETECT_POSE; 旧值 -0.3 不可达致 arm Timeout
    "watering": (-0.0, -0.05, "LEFT", "UP"),
    "shooting": (-0.25, -0.2, "LEFT", "DOWN"),
    "harvesting": (-0.0, 0, "LEFT", "UP"),
    "sorting": (-0.3, -0.05, "RIGHT", "UP"),
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
    # ordering 结束后: 前进 1.8m, 再顺时针原地转 120° (与 lane-stop 标定一致)
    # 任务结束重置里程计 (覆盖 sorting 默认清零钩子, 统一每任务清零;
    # 触发距离都是相对本次巡线起点的, 不受影响)
    car.reset_position()
    car.get_odometry(True)
    car.get_distance(True)
    print(f"[{task_name}] 里程计已重置")
    # 部分任务结束后沿逆时针转 (起步巡线前先调整朝向)
    turn_rad = _TURN_LEFT_RAD.get(task_name)
    if turn_rad is not None:
        print(f"[{task_name}] 沿逆时针转 {math.degrees(turn_rad):.0f}° (起步巡线)")
        car.move_for([0.0, 0.0, turn_rad], max_velocities=[0.10, 0.10, math.pi / 6])


class _CompetitionOrchestrator(Orchestrator):
    """等待态按键语义（点按）:

        4 = 全量开始   清空上一轮进度，从第 1 个任务跑完整轮
        1 = 跳过任务 1 开始  只跳过任务 run 逻辑, 仍正常巡线到任务点停
        2 = 跳过任务 1、2 开始（暂保留原逻辑, 待他人修改）
        3 = 立即停止并重启 run.py(刹停+释放硬件+exec 重跑), 全新 init 回初始状态

    3 在按键采集线程里全局拦截, 任何状态(等待/巡航/任务执行中)都立即生效;
    1 在任务执行中仍是"跳过当前任务"。
    """

    KEY_SKIP2 = 2  # 等待态: 跳过前 2 个任务开始

    def __init__(self, car):
        super().__init__(car)
        # 本轮"巡航通过"的任务集合: 正常巡线到任务点停, 只跳过 run(car) 逻辑
        self.run_skip: set = set()
        # 任务执行/巡航中按 3(急停)后置位: 回到等待态时先补做"重新初始化"
        self._reinit_pending = False

    def _reset_round(self) -> None:
        """清空本轮进度与结果，回到"从未开始"状态(下次按 4/1/2 都是新的一轮)."""
        self.done.clear()
        self.run_skip.clear()
        try:
            self._runner.results.clear()
        except Exception:
            pass
        print("本轮进度与结果已清空")

    def _reinit(self) -> None:
        """立即重新初始化 —— 与 create_car(reset=True) 一致:
        蜂鸣提示 + 机械臂复位(竖直/水平回家位) + 里程计清零, 再清空本轮进度。
        相机/推流/推理后端不重启, 只把车体状态和一轮新进度复位成"刚开机"。
        """
        try:
            self.car.beep()
            self.car.arm.reset_position()
            self.car.reset_position()
        except Exception as e:
            print(f"重新初始化异常: {e}")
        self._reset_round()

    def _restart_process(self) -> None:
        """按 3 全局急停: 刹停 + 释放硬件 + 关闭全部 fd, 再 exec 同参数重跑。

        任务 run() 里有不查 _stop_flag 的开环动作(move_time/set_velocity_for_duration),
        光设标志停不住; 直接重启进程才能"立即停一切 + 全新 init 回初始状态"。
        exec 保持同一 PID, systemd 服务不受影响, 重启后从等待态开始。
        """
        print("=== 按 3: 立即停止并重启（回到开机初始状态，像全新的一样）===")
        try:
            self.car._stop_flag = True
            self.car.stop()  # 立即刹停(发零速)
        except Exception as e:
            print(f"刹停异常: {e}")
        try:
            self.car.close()  # 释放相机/推流/按键线程
        except Exception as e:
            print(f"释放硬件异常: {e}")
        try:
            # 关掉所有非标准 fd(串口/相机/网络 socket), 避免 exec 后被继承占用
            os.closerange(3, os.sysconf("SC_OPEN_MAX"))
        except Exception:
            pass
        try:
            os.execv(
                sys.executable,
                [sys.executable, os.path.abspath(__file__)] + sys.argv[1:],
            )
        except Exception as e:
            print(f"重启失败: {e}")

    def _key_loop(self):
        """全局按键采集: 3 全局急停(任何状态立即重启), 其余键入队给编排器消费."""
        while self.running:
            try:
                event = self._key.read()
                if event and event <= 4:
                    if event == self.KEY_EMERGENCY:
                        # 3: 立即刹停并重启进程(无论等待/巡航/任务中)
                        self._restart_process()
                        # exec 失败回退: 3 仍入队, 由等待态/任务监听兜底处理
                    with self._key_lock:
                        self._key_queue.append(event)
            except Exception:
                pass
            time.sleep(0.02)

    def _listen_skip(self):
        """任务执行期间的按键: 1=跳过当前任务, 3=停止本轮并(回等待态后)重新初始化."""
        while self._skip_listener and not self._skip_listener.is_set():
            key = self._pop_key()
            if key == self.KEY_SKIP:
                print("=== 跳过当前任务 ===")
                self._skipped = True
                self.car.stop()
                return
            if key == self.KEY_EMERGENCY:
                print("=== 按 3: 停止本轮, 回到等待态后重新初始化 ===")
                self._emergency = True
                self._reinit_pending = True
                self.car._stop_flag = True
                self.car.stop()
                return
            time.sleep(0.02)

    def wait_start(self) -> None:
        """阻塞等待启动按键: 4 全量 / 1 跳过任务1 / 2 跳过任务1、2 / 3 重新初始化."""
        # 上一轮任务执行/巡航中按过 3 → 进等待态先补做"重新初始化"(像全新开机)
        if self._reinit_pending:
            self._reinit_pending = False
            print("=== 按 3: 重新初始化（回到初始状态，像全新的一样）===")
            self._reinit()
            print("重新初始化完成，等待一键启动...")
        print(
            "等待一键启动...（4 全量开始 / 1 跳过任务1开始 "
            "/ 2 跳过任务1、2开始 / 3 重新初始化）"
        )
        self._flush_keys()
        while self.running:
            key = self._pop_key()
            if key == self.KEY_START:  # 4: 全量开始
                self._reset_round()
                print("一键启动（全量）!")
                return
            if key == self.KEY_SKIP:  # 1: 跳过任务1开始(只跳过run逻辑, 仍巡线到点)
                self._reset_round()
                skipped = self.TASK_ORDER[:1]
                self.run_skip.update(skipped)
                print(
                    f"一键启动（跳过 {', '.join(skipped)} 的任务逻辑, "
                    "仍巡线到任务点停）!"
                )
                return
            if key == self.KEY_SKIP2:  # 2: 跳过前 2 个任务开始(原逻辑, 别人再改)
                self._reset_round()
                skipped = self.TASK_ORDER[:2]
                self.done.update(skipped)
                print(f"一键启动（跳过 {', '.join(skipped)}）!")
                return
            if key == self.KEY_EMERGENCY:  # 3: 立即重新初始化(像全新开机)
                print("=== 重新初始化（回到初始状态，像全新的一样）===")
                self._reinit()
                print("重新初始化完成，等待一键启动...")
                continue
            time.sleep(0.05)


def main():
    # --no-stream: 不启动 MJPEG 推流(省推流线程+每帧编码 CPU), 检测/巡线不受影响
    no_stream = "--no-stream" in sys.argv
    car = create_car(
        reset=True, comp_mode=True, stream=not no_stream
    )  # 初始化(含机械臂与里程计复位) + 比赛模式按键接管
    orch = _CompetitionOrchestrator(car)
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
