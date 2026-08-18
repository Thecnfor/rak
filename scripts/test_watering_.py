#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task2 (watering) 浇水 —— 移植 4_car task2_water_tower 主干(精简串行版)
==================================================================
参数/逻辑来自 4_car main/task/task2_water_tower.py + task_config.yml
(water_tower_task 段)。方法全部用本仓库现有 SDK(create_car 直连)。

移植要点:
  · 手爪刻度两车一致(UP=-90 MID=-37 DOWN=0), 4_car 度数原样搬
  · x/y 滑轨方向两车一致(0=右/底, 负=左/上), mm→m 除以 1000 即可
  · 大臂 4_car 度数(+90/-96) → 本仓库物理角度, 按场景映射:
        +90(init/pick 朝置物架侧) → +93
        -96(detection/carry 朝水塔侧) → -93
        ⚠️ 物理对应需现场验证
  · 砍掉 4_car 的 3阶段安全转位/X补走校验/视觉超时重试/并发, 只留主干
  · 底盘×机械臂全串行(单 MC602 总线防丢命令)
  · 转大臂前 XY 先到安全位(4_car 安全区 X∈[-300,-200] Y∈[-200,-90]mm)

运行: python scripts/test_watering_.py
急停: Ctrl+C
"""

import os, sys, time, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tasks.tools import create_car

# ========================== 参数 (来自 4_car task_config.yml, mm→m 已换算) ==========================
# ----- 底盘几何 -----
TOWER_SPACING  = 0.55      # 两塔中心间距 (m)
GROUP_FWD, GROUP_BACK = 0.35, 0.33   # 塔1向前/塔2向后 每组水块间距 (m)
CHASSIS_V      = [0.10, 0.10, math.pi / 3]  # move_for 速度上限 [前后, 横向, 转角]

# ----- 水塔等级标签 → 需搬水块数 -----
WATER_LABEL = {"water_l1": 1, "water_l2": 2, "water_l3": 3}
TARGET_WATER = "water"     # 视觉伺服/对齐用的目标类(水塔/水块)

# ----- 大臂角度 (本仓库物理角度, 由 4_car 度数按场景映射) -----
ARM_SHELF = +93   # 4_car +90: 朝置物架侧 (抓块)
ARM_TOWER = -93   # 4_car -96: 朝水塔侧 (识别/投放)

# ----- 姿态 (mm → m /1000) -----
DETECT_POSE  = dict(x=-0.200, y=-0.150, arm=ARM_TOWER, hand=-50)  # 进塔/识别前姿态 (4_car -60 现场+10)
DETECT_Y     = -0.010                        # 识别时 y 降到检测高度
PICK_POSE_Y  = -0.150                        # 抓块姿态 y (servo_y)
PICK_HAND    = 0                             # 抓块姿态手爪 (4_car -10 现场+10)
FIRST_CUBE_X, SECOND_CUBE_X = -0.145, -0.220 # 每块组内 第1/第2 块 X
GRASP_Y, LIFT_Y = -0.050, -0.150             # 吸块下降 y / 吸完抬回 y
DELIVER_Y    = [-0.010, -0.045, -0.085]      # 放块第1/2/3层 y (梯度)
DELIVER_HAND = [-70, -75, -75]               # 放块第1/2/3层手爪 (4_car -80/-85/-85 现场+10)
CARRY_X      = [[-0.060, -0.055, -0.055],
                [-0.060, -0.055, -0.055]]    # 每塔每块放块 X (m)

# ----- 转大臂前 XY 安全位 (4_car 安全区 X∈[-300,-200] Y∈[-200,-90]mm) -----
SAFE_X, SAFE_Y = -0.200, -0.150

# ----- 视觉伺服参数 (现场校准) -----
#   kp=(左右增益, 前后增益); sign=(左右符号, 前后符号)。
#   TRACK: kp_x=0=横向锁死, kp_y=0.22=只前后; sign_y=-1=前后方向(反则"只后退", 现场定)
TRACK = dict(                                   # 底盘对齐水塔 (只前后)
    cx=0.142, cy=0.183, kp=(0.0, 0.22),
    sign=(1.0, -1.0), deadband=0.02, hold=6,
    v_max=0.11, timeout=15.0,
)
PICK = dict(                                    # 机械臂伺服抓水块
    cx=0.098, cy=-0.398, gains=(0.1, 0.1),
    sign=(-1.0, 1.0), deadzone=0.03, settle=6,   # 大臂-1/滑轨+1 (跟 seeding 抓取一致)
    timeout=15.0,
)


# ========================== 辅助函数 ==========================
def _chassis(car, pos, target):
    """底盘纵向 move_for 到相对位移 target(m), 自记账(相对塔原点), 不依赖 odom 绝对值."""
    dx = target - pos[0]
    if abs(dx) < 0.05:
        pos[0] = target
        return
    car.move_for([dx, 0, 0], max_velocities=CHASSIS_V)
    pos[0] = target


def _arm_to(car, x, y, arm, hand):
    """切机械臂姿态: 先抬Y到安全高 → 收X到安全位(XY 冻结) → 转大臂/手爪 → XY 并发到位.
    保留 4_car "转大臂时 XY 冻结在安全区" 的顺序防撞; 先抬 Y 再收 X, 防低 Y 时转臂撞塔."""
    car.arm.move_y_position(SAFE_Y)
    car.arm.move_x_position(SAFE_X)
    car.arm.set_arm_angle(arm)
    if hand is not None:
        car.arm.set_hand_angle(hand)
    car.arm.goto_position(x, y)


# 底盘对齐/识别都算"水"的标签: water(塔/水块) 或 water_l*(等级标), 哪个可见用哪个
WATER_ALIGN_LABELS = ("water", "water_l1", "water_l2", "water_l3")


def _find_water_label(car, max_age=0.5):
    """取实时缓存里任一可见的水标签作为对齐目标(water 或 water_l* 都算)."""
    for d in car.get_realtime_detections(max_age=max_age):
        if d[2] in WATER_ALIGN_LABELS:
            return d[2]
    return None


def _align_tower(car):
    """底盘视觉对齐水塔: 目标取实时缓存里任一可见水标签(先等 1.5s 让其出现),
    只前后(横向锁死), 拉到 setpoint(0.142,0.183)."""
    end = time.time() + 1.5
    label = None
    while time.time() < end:
        label = _find_water_label(car)
        if label is not None:
            break
        time.sleep(0.05)
    if label is None:
        print("[底盘] 1.5s 内未见任何水标签, 跳过对齐")
        return
    print(f"[底盘] 对齐目标: {label}")
    car.chassis_align(label, cx=TRACK["cx"], cy=TRACK["cy"],
                      kp=TRACK["kp"], sign=TRACK["sign"],
                      deadband=TRACK["deadband"], hold=TRACK["hold"],
                      v_max=TRACK["v_max"], decouple_xy=False,
                      timeout=TRACK["timeout"])


def _detect_water_num(car, timeout=1.0):
    """从侧视实时缓存识别水塔等级标 water_l*, 返回需搬块数(没识别到返回 0)."""
    end = time.time() + timeout
    while time.time() < end:
        for d in car.get_realtime_detections(max_age=0.5):
            if d[2] in WATER_LABEL:
                return WATER_LABEL[d[2]], d[2]
        time.sleep(0.05)
    return 0, None


def _servo_pick(car):
    """车不动, 机械臂视觉伺服把水块对齐到 setpoint(0.098,-0.398) → 下探吸 → 抬回."""
    car.arm_servo_align(TARGET_WATER, cx=PICK["cx"], cy=PICK["cy"],
                        gains=PICK["gains"], sign=PICK["sign"],
                        deadzone=PICK["deadzone"], settle=PICK["settle"],
                        timeout=PICK["timeout"])
    car.arm.set_hand_angle(10)             # 下降时手爪转朝下 (4_car descend_hand=0 现场+10)
    car.arm.move_y_position(GRASP_Y)       # 降到水块高度
    car.arm.grasp(True)                    # 吸气吸住
    car.arm.move_y_position(LIFT_Y)        # 抬回运输高度


def run_one_tower(car, tower_idx):
    """处理一座水塔: 识别水量 → 逐块「抓→放」. 底盘相对塔原点, 串行."""
    # 识别
    car.arm.move_y_position(DETECT_Y)      # y 降检测高度
    _align_tower(car)                      # 底盘只前后对齐水塔
    n, label = _detect_water_num(car)      # water_l* → 需搬块数
    print(f"\n[水塔{tower_idx+1}] label={label}, 水量={n}")
    car.beep()

    pos = [0.0]                            # 底盘相对塔原点 (m)
    direction = 1.0 if tower_idx == 0 else -1.0   # 塔1水块在前方(向前拿), 塔2在后方(向后拿)
    for k in range(n):
        print(f"  搬第 {k+1}/{n} 块")
        # ===== 抓 =====
        group = k // 2                     # 每 2 块一组
        dist = direction * group * (GROUP_FWD if direction > 0 else GROUP_BACK)
        _chassis(car, pos, dist)                       # 底盘到该组
        pick_x = FIRST_CUBE_X if k % 2 == 0 else SECOND_CUBE_X
        _arm_to(car, pick_x, PICK_POSE_Y, ARM_SHELF, PICK_HAND)  # 切抓姿(朝置物架)
        _servo_pick(car)                                 # 视觉伺服抓块
        # ===== 放 =====
        _chassis(car, pos, 0.0)                          # 底盘回塔
        _arm_to(car, CARRY_X[tower_idx][k], DELIVER_Y[k],
                ARM_TOWER, DELIVER_HAND[k])              # 切投放(朝水塔, 梯度深度)
        car.arm.grasp(False)                             # 放气投放
        car.beep()


def main():
    car = create_car()
    try:
        # 切检测姿态 (4_car entry_back_off 的底盘回退已删除: 直接原地切臂姿)
        _arm_to(car, DETECT_POSE["x"], DETECT_POSE["y"],
                DETECT_POSE["arm"], DETECT_POSE["hand"])

        print("===== 第 1 个水塔 =====")
        run_one_tower(car, tower_idx=0)

        # 塔间前进 + 切回检测姿态
        print(f"\n===== 前进 {TOWER_SPACING} m 到第 2 个水塔 =====")
        car.move_for([TOWER_SPACING, 0, 0], max_velocities=CHASSIS_V)
        _arm_to(car, DETECT_POSE["x"], DETECT_POSE["y"],
                DETECT_POSE["arm"], DETECT_POSE["hand"])

        print("===== 第 2 个水塔 =====")
        run_one_tower(car, tower_idx=1)

        print("\n全部完成 ✓")
        car.beep(); car.beep()
    except KeyboardInterrupt:
        print("\n急停")
    finally:
        car.arm.grasp(False)
        car.stop()


if __name__ == "__main__":
    main()
