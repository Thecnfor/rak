#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务4: 作物采收 — 采收区 2 色 4cm 球(蓝/黄), 抓一个放一个。

流程结构移植自 4_car rak-car c3e6ab5 的 task4:
    摆抓取起手 → 第一球(蠕动找球 → 底盘对齐最左球 → 臂伺服 → 抓放) →
    后续球(前进 0.08m→臂回起手 → 找球≤3s → 臂伺服 → 抓放) →
    前进≥N 且连续无球退出。

放球统一简化: 抓取后不开存储架、不送仓。臂抬到 LIFT_Y(-0.15) 后大臂转到
-80° 即放气, x 与手爪保持不动, 视为完成一次。后续每颗球同样处理。

方法全部复用本仓现有: arm_servo_align(纯臂闭环) / chassis_align(底盘对齐,
已补 prefer_left) / lane_dis_offset / set_arm_pose / grasp。

⚠️ 坐标/伺服参数均为占位或沿用旧 harvesting.py 现值, 标「需重标」的必须现场
   重新标定。本仓 Y 轴「向下为正, 0=最底(下限位), -0.2=最顶」, 与 rak-car
   的 mm 坐标系方向相同但量纲/物理行程不同。
"""
import time

# ── 标签 ────────────────────────────────────────────────────────────
BALLS = ("ball_blue", "ball_yellow")

# ── 吸嘴 setpoint(目标在吸嘴正下方时 bbox 中心, 归一化) — 需重标 ──
NOZZLE = {
    "ball_blue": (0.0, -0.2),
    "ball_yellow": (0.0, -0.2),
}

# ── 姿态(x/y 米; arm/hand 角度) ─────────────────────────────────────
#   Y 轴: 向下为正, 0=最底(下限位), -0.2=最顶(抬升为负值)
#   大臂角度: 93=向左 / 0=中 / -93=向右; 末端角度: -90=收起 / -37=半 / 0=朝下
PICK_POSE = dict(x=-0.05, y=-0.15, arm=-90, hand=10)  # 抓取起手 — 需重标
RELEASE_ARM = -80     # 放球大臂角: 生成后转 -80 即放气(用户 2026-08-18 拍板)
GRASP_Y, LIFT_Y = 0.05, -0.15   # 降到抬 5cm 贴球面吸 / 抬回(吸球不触底, 用户 2026-08-18 拍板)

# ── 伺服参数 — 需重标 ────────────────────────────────────────────────
#   sign 沿用 seeding 的 PICK_SERVO(-1,1); 现场定方向(越对越偏就取反)
BALL_SERVO = dict(gains=(0.2, 0.1), sign=(-1.0, 1.0), deadzone=0.02,
                  timeout=8.0, debug=True)
#   chassis_align 默认参数(交叉映射已在方法内处理), 未在实车验证 — 现场测
CHASSIS_ALIGN = dict(kp=(0.10, 0.10), sign=(-1.0, 1.0), deadband=0.05,
                     hold=6, v_max=0.12, timeout=7.0)

# ── 扫描/退出 ────────────────────────────────────────────────────────
CREEP_V = 0.05        # 蠕动速度(m/s); rak-car 0.05
CREEP_MAX_M = 1.0     # 第一球蠕动距离占位上限(m) — 现场定(vision 触发已见球, 可调小)
ADVANCE_M = 0.08      # 每球前进距离(m); rak-car 0.08
ADVANCE_SPEED = 0.3   # 前进巡线速度(m/s)
LOOK_S = 3.0          # 每站找球上限(s); rak-car 3.0
MIN_ADVANCES = 7      # 前进≥N 次才允许按「连续无球」退出; rak-car 7
EMPTY_ROUNDS = 2      # 连续无球轮数 → 收工; rak-car 2


def _has(car, label, max_age=0.3):
    """只查不移动: 侧视实时缓存里是否存在该 label(供找球/扫描用)."""
    return any(d[2] == label for d in car.get_realtime_detections(max_age=max_age))


def _leftmost_ball(car, max_age=0.3):
    """从侧视缓存里挑最左(画面 x_c 最小)的球, 返回 label 或 None."""
    dets = [d for d in car.get_realtime_detections(max_age=max_age) if d[2] in BALLS]
    if not dets:
        return None
    return min(dets, key=lambda d: d[4])[2]


def _look_ball(car, timeout=LOOK_S, max_age=0.3):
    """找球≤timeout: 轮询侧视缓存直到见到球, 返回最左球 label 或 None.

    移植 rak-car _look_grabbable_ball 的「窗口内取最左」语义; 这里不做可抓窗口
    过滤(本仓 arm_servo_align 自带 deadzone/lock, 球太远会超时→算空轮跳过).
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if getattr(car, "_stop_flag", False):
            return None
        label = _leftmost_ball(car, max_age)
        if label is not None:
            return label
        time.sleep(0.1)
    return None


def _ensure_hand(car, target=10.0, retries=3, settle=0.5):
    """视觉对齐前强制末端手爪到位(舵机无回读, 连发+等到位兜底, 同 seeding)."""
    for _ in range(retries):
        car.arm.set_hand_angle(target)
        time.sleep(settle)


def _creep_until_ball(car, speed=CREEP_V, max_distance=CREEP_MAX_M,
                      timeout=40.0, max_age=0.3):
    """慢速前移直到见到球(蓝/黄)或走满距离/超时. 返回是否见到球.

    移植 rak-car _CreepThread: MC602 速度命令点动式, 循环内须持续重发
    set_velocity; 距离只认里程计增量.
    """
    start = car.get_distance()
    t0 = time.monotonic()
    car.set_velocity(speed, 0.0, 0.0)
    try:
        while time.monotonic() - t0 < timeout:
            if getattr(car, "_stop_flag", False):
                return False
            if any(_has(car, l, max_age) for l in BALLS):
                return True
            if car.get_distance() - start >= max_distance:
                break
            time.sleep(0.1)
    finally:
        car.set_velocity(0.0, 0.0, 0.0)
    return False


def _pick(car, label):
    """臂伺服对齐 → 降到抬5cm贴球面吸 → 抬升 → 大臂转-80° 放气(不开仓不送 bin)."""
    _ensure_hand(car)  # ① 视觉对齐前: 强制末端到位
    # 机械臂视觉伺服(纯臂闭环, 只动大臂+滑轨); 超时也照样盲抓(对齐尽力)
    if not car.arm_servo_align(label, *NOZZLE[label], prefer_left=True, **BALL_SERVO):
        print(f"[harvest] {label} 臂伺服未收敛, 照样盲抓")
    _ensure_hand(car)  # ② 抓取前兜底: 滑轨/Y 大电流移动可能把舵机打回
    car.arm.move_y_position(GRASP_Y)   # 降到抬5cm, 贴球面
    car.arm.grasp(True)                # 吸气
    time.sleep(0.5)                    # 吸住保持
    car.arm.move_y_position(LIFT_Y)    # 抬到 -0.15
    # 放球: 大臂转 -80° 即放气, x 与手爪保持不动(统一简化, 用户 2026-08-18 拍板)
    car.arm.set_arm_angle(RELEASE_ARM)
    car.arm.grasp(False)               # 放气


def run(car):
    picks = 0

    # ── 开始阶段(编排器已用 vision 巡线到球触发点, 不开存储架) ──
    car.arm.set_arm_pose(PICK_POSE["x"], PICK_POSE["y"],
                         PICK_POSE["arm"], PICK_POSE["hand"])

    # ── 第一球: 蠕动找球 → 底盘对齐最左球 → 抓放 ──
    if not _creep_until_ball(car):
        print("[harvest] 第一球蠕动未见球, 收工")
        return picks
    label = _leftmost_ball(car)
    if label is None:
        print("[harvest] 蠕动后仍无球, 收工")
        return picks
    print(f"[harvest] 首个球: {label}")
    # 底盘对齐最左球(未验证, 现场测 sign/收敛); 失败不阻塞, 臂伺服兜底
    car.chassis_align(label, prefer_left=True, **CHASSIS_ALIGN)
    _pick(car, label)
    picks += 1

    # ── 后续球循环: 前进 0.08m∥臂回起手 → 找球≤3s → 抓放 ──
    advances = 0
    empty = 0
    while True:
        if getattr(car, "_stop_flag", False):
            break
        # 前进 + 臂回起手(串行, 避免抢串口; 要并发可后续用线程包 set_arm_pose)
        car.lane_dis_offset(ADVANCE_SPEED, ADVANCE_M)
        car.arm.set_arm_pose(PICK_POSE["x"], PICK_POSE["y"],
                             PICK_POSE["arm"], PICK_POSE["hand"])
        advances += 1
        label = _look_ball(car)
        if label is None:
            empty += 1
            print(f"[harvest] 第{advances}站未见可抓球, 空轮 {empty} 次")
            if advances >= MIN_ADVANCES and empty >= EMPTY_ROUNDS:
                print(f"[harvest] 前进{advances}次, 连续{empty}轮无球, 收工")
                break
            continue
        empty = 0
        print(f"[harvest] 第{advances}站锁定 {label}")
        _pick(car, label)
        picks += 1

    # ── 收尾(不开存储架, 无需放下) ──
    return picks
