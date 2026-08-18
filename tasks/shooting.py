#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务三射击: 对确认害虫的板位依次对齐击发 (rak 原生实现).

target_detection 对每块目标板做 LLM 判定写入 animal_list: 值为 0 表示该板确认是
害虫需射击, 非 0 跳过; 无确认害虫直接返回不动车。

对齐/步进/击发全部走 MyCar 既有设施 (move_to_detection_target + lane_dis_offset +
car.shooting()), 数值为现场标定值, 改动需重新标定。
"""
import time

STEP = 0.14     # 相邻目标板间距 (m)
D_X = 0.2       # 射击对齐距离: 目标板到正前方该距离时击发
ROW_END = 0.48  # 4 块板总宽 = 3 * STEP, 射完补偿推进到排尾


def run(car, animal_list=None):
    """依次对齐击发所有确认害虫 (animal_list 中值为 0 的板位)."""
    if animal_list is None:
        animal_list = [0, 0, 0, 0]  # 未传识别结果 → 视为全部为害虫

    targets = [i for i, v in enumerate(animal_list) if v == 0]
    if not targets:
        print("[shooting] 无确认害虫, 跳过射击", flush=True)
        return

    car.arm.set_arm_pose(arm="LEFT", hand="UP", x=-0.25, y=-0.04)

    def align():
        move_to_detection_target(
            delta_x=0.02, delta_y=None, sort_pos=(0, 0), score_thresh=0.8
        )

    align()  # 先对齐排首可见板, 建立射击基线
    prev = 0
    for idx in targets:
        car.lane_dis_offset(speed=0.2, dis_hold=(idx - prev) * STEP)  # 推进到本板
        align()                                                        # 对准
        time.sleep(1)
        car.shooting()                                                 # 击发 (自带蜂鸣提示)
        time.sleep(1)
        prev = idx

    # 射完推进到排尾, 为后续巡线留位
    car.lane_dis_offset(speed=0.2, dis_hold=ROW_END - targets[-1] * STEP)
