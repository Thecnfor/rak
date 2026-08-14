#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
最小机械臂大臂舵机测试: 只发大臂角度命令, 不初始化 X/Y 轴/编码器。

两种用法(都必须在 Jetson 真机上跑, 开发机会卡在"未接控制器"死循环):
1) 独立脚本:
       python3 scripts/test_arm_angle.py            # 默认 RIGHT(-93°)
       python3 scripts/test_arm_angle.py LEFT       # LEFT(93°)
       python3 scripts/test_arm_angle.py MID        # MID(0°)
       python3 scripts/test_arm_angle.py 45         # 任意角度
       python3 scripts/test_arm_angle.py --no-hand  # 跳过抬手安全步骤
2) 经 run.py 调用(等价):
       python run.py arm_test
       python run.py arm_test LEFT --no-hand

注意:
- 导入 controller_wrap 即探测串口(serial_wrap 单例), 连不上控制器会一直阻塞打印。
- set_angle 只等串口应答、不等舵机物理到位, 函数用 sleep 兜底等它转完。
- 本脚本没有急停线程(急停在 MyCar 里), 转臂前确认周围无障碍。
- 抬手步骤对应 arm_cfg.yaml hand2 的 UP(-90°), 与 reset_position 一致(先抬手再转臂)。
"""

import argparse
import os
import sys
import time

# 让脚本在任意 cwd 下都能 import smartcar(仓库根目录加入 sys.path)
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, repo_root)

# 导入即探测串口并确定 ctl_id(MC601/602), 连不上控制器会阻塞 —— 必须在真机上跑
from smartcar.whalesbot.vehicle.base.controller_wrap import ServoBus, ServoPwm

# 与 arm_cfg.yaml 的 hand.angle_list 保持一致
ANGLE_MAP = {"LEFT": 93, "MID": 0, "RIGHT": -93}
SPEED = 80  # 舵机转速, 与 set_arm_angle 默认一致


def run_arm_test(angle="RIGHT", speed=SPEED, no_hand=False):
    """
    只发大臂角度命令(不初始化整机)。

    Args:
        angle: LEFT/MID/RIGHT 或角度数字, 默认 RIGHT
        speed: 舵机转速
        no_hand: True 时跳过抬手安全步骤

    Returns:
        int: 实际下发的大臂角度(字符串名解析后的数值)
    """
    target = angle.upper() if isinstance(angle, str) else str(angle)
    resolved = ANGLE_MAP.get(target)
    if resolved is None:
        resolved = int(angle)  # 数字角度; 非法输入抛 ValueError

    if not no_hand:
        # 安全步骤: 先把手指抬起来(UP=-90), 转臂时手部别扫到东西
        hand = ServoPwm(2, mode=180)
        hand.set_angle(-90, speed)
        time.sleep(1)

    arm = ServoBus(2)  # 大臂总线舵机, 端口 2(来自 hand_cfg.hand.port)
    print(f"arm -> {resolved}° (speed {speed})")
    arm.set_angle(resolved, speed)
    time.sleep(1.5)  # 命令不等到位, 手动等舵机转完
    print("done")
    return resolved


def hold_arm_test(angle="RIGHT", speed=SPEED, no_hand=False):
    """
    持续重发大臂角度命令并保持进程存活, 检测舵机是否上锁/执行。

    WhalesBot 部分设备(如 X 轴)需持续发命令才执行/保持, 单条命令可能不够。
    运行后请用手感受大臂是否变硬(上锁)或开始转动, Ctrl+C 退出。
    """
    target = angle.upper() if isinstance(angle, str) else str(angle)
    resolved = ANGLE_MAP.get(target)
    if resolved is None:
        resolved = int(angle)

    if not no_hand:
        hand = ServoPwm(2, mode=180)
        hand.set_angle(-90, speed)
        time.sleep(1)

    arm = ServoBus(2)
    print(f"持续重发 arm -> {resolved}° (每0.5s, Ctrl+C 退出)...")
    print("观察: 大臂是否上锁/转动?")
    try:
        while True:
            arm.set_angle(resolved, speed)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("stopped")


def sweep_servo_test(speed=SPEED, no_hand=False):
    """
    扫描 0-8 号总线舵机地址, 找出真正带大臂的那个口。

    逐口发 60°→0° 动作, 你观察大臂在哪个口会动; 其他口无舵机则无反应。
    """
    if not no_hand:
        hand = ServoPwm(2, mode=180)
        hand.set_angle(-90, speed)
        time.sleep(1)

    print("扫描总线舵机地址 0-8, 观察大臂在哪个口动作...")
    for port in range(0, 9):
        srv = ServoBus(port)
        print(f"  口 {port}: 转 60°")
        srv.set_angle(60, speed)
        time.sleep(1.2)
        print(f"  口 {port}: 回 0°")
        srv.set_angle(0, speed)
        time.sleep(1.2)
    print("扫描结束")


def main():
    parser = argparse.ArgumentParser(description="最小大臂舵机测试")
    parser.add_argument(
        "angle", nargs="?", default="RIGHT",
        help="LEFT/MID/RIGHT 或角度数字, 默认 RIGHT",
    )
    parser.add_argument("--no-hand", action="store_true", help="跳过抬手安全步骤")
    parser.add_argument(
        "--hold", action="store_true",
        help="持续重发命令并保持进程存活(检测舵机是否上锁)",
    )
    parser.add_argument(
        "--sweep", action="store_true",
        help="扫描 0-8 号总线舵机地址, 找出带大臂的口",
    )
    args = parser.parse_args()

    if args.sweep:
        sweep_servo_test(no_hand=args.no_hand)
    elif args.hold:
        hold_arm_test(angle=args.angle, no_hand=args.no_hand)
    else:
        run_arm_test(angle=args.angle, no_hand=args.no_hand)


if __name__ == "__main__":
    main()
