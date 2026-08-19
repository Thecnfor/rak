#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂两种姿态切换测试 — 每次运行只摆一个姿势.

初始化后(create_car reset=True: 复位臂+清里程计)把臂摆到指定姿态,
打印命令值 + 回读值(滑轨 x/y 是真编码器回读, 大臂/末端是最近命令值,
舵机无位置回读), 然后保持该姿势不动(侧视 cam2 推流由 create_car 自动开启,
浏览器可看实时画面)。

用法:
    python scripts/test_arm_pose.py 1        # 姿势1 竖拍: x=-0.20 y=-0.02 arm=-93 hand=-70
    python scripts/test_arm_pose.py 2        # 姿势2 横拍: x=-0.22 y=-0.15 arm=+93 hand=-20
    python scripts/test_arm_pose.py --pose 1 # 等价

退出: Ctrl+C (保持当前姿势, 不再动)
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 两个测试姿势 (x/y 米, arm/hand 度)
POSES = {
    1: dict(x=-0.20, y=-0.02, arm=-93, hand=-70, name="竖拍"),
    2: dict(x=-0.22, y=-0.15, arm=+93, hand=-20, name="横拍"),
}


def main():
    ap = argparse.ArgumentParser(description="机械臂姿态切换测试(每次单姿势)")
    ap.add_argument("pose", nargs="?", type=int, choices=(1, 2), help="姿势号 1/2")
    ap.add_argument("--pose", dest="pose_opt", type=int, choices=(1, 2),
                    help="或 --pose 1/2 (与位置参数二选一)")
    args = ap.parse_args()
    n = args.pose if args.pose is not None else args.pose_opt
    if n is None:
        n = 1
    p = POSES[n]

    from tasks.tools import create_car
    car = create_car()          # reset=True: 蜂鸣 + 机械臂复位 + 里程计清零
    try:
        print(f"\n===== 姿势{n} {p['name']} =====")
        print(f"命令: x={p['x']:.3f}  y={p['y']:.3f}  arm={p['arm']:+.0f}°  hand={p['hand']:+.0f}°")
        car.arm.set_arm_pose(p["x"], p["y"], p["arm"], p["hand"])
        time.sleep(1.5)          # 等滑轨/舵机到位
        rx = car.arm.x_get_position()
        ry = car.arm.y_get_position()
        print(f"回读: x={rx:.3f}  y={ry:.3f}  arm={car.arm.angle:+.1f}°  "
              f"hand={car.arm.hand_angle:+.1f}°  (arm/hand 为最近命令值)")
        ok = abs(rx - p["x"]) < 0.02 and abs(ry - p["y"]) < 0.02
        print(f"[{'PASS' if ok else '注意'}] 滑轨 XY 到位 {'OK' if ok else '偏差偏大(可能仍在移动/丢帧)'}")
        print("\n保持该姿势不动。侧视 cam2 推流已自动开启, 浏览器看: "
              "http://<jetson-ip>:<port>/stream/ 的 cam2")
        print("Ctrl+C 退出")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n退出(保持当前姿势)")
    finally:
        car.stop()
        car.close()


if __name__ == "__main__":
    main()
