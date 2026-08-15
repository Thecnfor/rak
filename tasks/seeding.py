#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务1: 自动移苗(播种) — 三列育苗筒 -> 种植槽. 逻辑迁移自 4_car task1_seeding.

与 4_car 相比只补了一个精简版机械臂视觉伺服(_servo), 其余全部映射到现有 SDK。
⚠️ 所有数值/姿态/setpoint 是 4_car 的标定占位, 必须按本车重标定。
   本车 y 轴约定与 4_car 相反: y 正 = 抬升, y=0 = 最低。
"""
import math
import time

# ── 底盘列(相对位移 m) & 标签→槽映射 ──────────────────────────────
SOURCE = {1: 0.0, 2: 0.15, 3: 0.30}
SLOT = {1: 0.0, 2: 0.15, 3: 0.30}
TARGET_SLOT = {"cylinder_1": 3, "cylinder_2": 2, "cylinder_3": 1}  # 左→右由大到小(1大→槽3)
CYLINDERS = ("cylinder_1", "cylinder_2", "cylinder_3")

# ── 吸嘴 setpoint(目标在吸嘴正下方时 bbox 中心, 归一化) — 需重标 ──
NOZZLE = {
    "cylinder_1": (0.050, -0.425),
    "cylinder_2": (0.140, -0.420),
    "cylinder_3": (0.120, -0.410),
}
MARKER = "cylinder_set"
MARKER_NOZZLE = (0.072, -0.331)

# ── 姿态(arm: LEFT/MID/RIGHT; x/y 米, y 正=抬升) — 需重标 ──────────
PICK_POSE = dict(x=0.0, y=0.2, arm="LEFT", hand="DOWN")
PLACE_POSE = dict(x=0.0, y=0.2, arm="RIGHT", hand="DOWN")
GRASP_Y, LIFT_Y = 0.0, 0.2          # 降到底吸 / 抬回
PLACE_Y, PLACE_LIFT_Y = 0.02, 0.04  # 放苗降 / 释放后抬离(防拖拽)

MOVE_V = 0.1  # 底盘平移限速, 降漂移


# ── 伺服参数(抓/放分开, 来自 4_car task_config.yml) ───────────────────
PICK_SERVO = dict(gains=(0.5, 0.30), sign=(1.0, -1.0), deadzone=0.05)
PLACE_SERVO = dict(gains=(0.3, 0.2), sign=(1.0, 1.0), deadzone=0.06)


def _find(car, label, setpoint=None, max_age=0.3):
    """侧视实时缓存里找目标; 多个时取离 setpoint 最近. 返回 (cx, cy) 归一化中心."""
    dets = [d for d in car.get_realtime_detections(max_age=max_age) if d[2] == label]
    if not dets:
        return None
    if setpoint:
        sx, sy = setpoint
        dets.sort(key=lambda d: (d[4] - sx) ** 2 + (d[5] - sy) ** 2)
    d = dets[0]
    return d[4], d[5]


def _servo(car, label, setpoint, params, settle=3, timeout=4.0):
    """机械臂 2D 视觉伺服: 大臂角控 cx + X 滑轨控 cy → 吸嘴 setpoint.

    params 含 gains/sign/deadzone; sign 为方向符号, 越对越偏就取反。
    """
    gains, sign, deadzone = params["gains"], params["sign"], params["deadzone"]
    end = time.time() + timeout
    hits = 0
    while time.time() < end:
        p = _find(car, label, setpoint)
        if p is None:
            hits = 0
            car.arm.x_speed(0)
            time.sleep(0.02)
            continue
        e_cx, e_cy = setpoint[0] - p[0], setpoint[1] - p[1]
        if abs(e_cx) < deadzone and abs(e_cy) < deadzone:
            hits += 1
            if hits >= settle:
                car.arm.x_speed(0)
                return True
        else:
            hits = 0
            car.arm.set_arm_angle(car.arm.angle + sign[0] * gains[0] * e_cx)
            car.arm.x_speed(sign[1] * gains[1] * e_cy)
        time.sleep(0.03)
    car.arm.x_speed(0)
    return False


def _pick(car, label):
    if not _servo(car, label, NOZZLE[label], PICK_SERVO):
        raise RuntimeError(f"pick {label} 未收敛")
    car.arm.move_y_position(GRASP_Y)
    car.arm.grasp(True)
    car.arm.move_y_position(LIFT_Y)


def _place(car):
    _servo(car, MARKER, MARKER_NOZZLE, PLACE_SERVO)  # 未收敛也放, 完赛优先
    car.arm.move_y_position(PLACE_Y)
    car.arm.grasp(False)
    car.arm.move_y_position(PLACE_LIFT_Y)


def _chassis(car, target, pos):
    """闭环 move_for 到相对位移 target(m), 自记账, 不依赖 odom 绝对值."""
    dx = target - pos[0]
    if abs(dx) < 0.05:
        return
    car.move_for([dx, 0.0, 0.0], max_velocities=[MOVE_V, MOVE_V, math.pi / 3])
    pos[0] = target


def run(car):
    pos = [0.0]           # 底盘纵向自记账
    seen = None
    completed = []
    for col in (1, 2, 3):
        _chassis(car, SOURCE[col], pos)
        car.arm.set_arm_pose(PICK_POSE["x"], PICK_POSE["y"],
                             PICK_POSE["arm"], PICK_POSE["hand"])
        # 扫描本列 cylinder; 没有就用剩余 label 兜底
        label = next((l for l in CYLINDERS if _find(car, l)), None)
        if label is None:
            label = next((l for l in CYLINDERS if l not in completed), None)
            if label is None:
                continue
        # 1↔3 纠错(同尺寸易认错)
        if seen is None:
            seen = label
        elif label == seen and seen in ("cylinder_1", "cylinder_3"):
            label = "cylinder_3" if seen == "cylinder_1" else "cylinder_1"
        _pick(car, label)
        completed.append(label)
        # 放苗: 底盘到槽列 + 切放苗姿态
        _chassis(car, SLOT[TARGET_SLOT[label]], pos)
        car.arm.set_arm_pose(PLACE_POSE["x"], PLACE_POSE["y"],
                             PLACE_POSE["arm"], PLACE_POSE["hand"])
        _place(car)
    return completed
