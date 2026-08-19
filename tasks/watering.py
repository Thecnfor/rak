#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task2 (watering) 浇水 ——

  · 手爪刻度两车一致(UP=-90 MID=-37 DOWN=0), 4_car 度数原样搬
  · x/y 滑轨方向两车一致(0=右/底, 负=左/上), mm→m 除以 1000 即可
  · 大臂 4_car 度数(+90/-96) → 本仓库物理角度, 按场景映射:
        +90(init/pick 朝置物架侧) → +93
        -96(detection/carry 朝水塔侧) → -93
  · 砍掉 4_car 的 3阶段安全转位/X补走校验/视觉超时重试/并发, 只留主干
  · 底盘×机械臂全串行(单 MC602 总线防丢命令)
  · 转大臂前 XY 先到安全位(4_car 安全区 X∈[-300,-200] Y∈[-200,-90]mm)
  · 识别姿势(x=-0.2,y=-0.02,大臂-93,末端-70) = target_detection 任务的结束钉姿势
    (run.py TASK_END_POSE), run() 只 _check_detect_pose 校验, 不在才安全兜底摆回
运行: python tasks/watering.py    (独立自建车; 编排经 run.py 走 run(car) 同一流程)
急停: Ctrl+C
"""

import os, sys, time, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tasks.tools import create_car

# ========================== 参数 (来自 4_car task_config.yml, mm→m 已换算) ==========================
# ----- 底盘几何 -----
TOWER_SPACING  = 0.60     # 两塔中心间距 (m)
GROUP_FWD, GROUP_BACK = 0.34, 0.34   # 塔1向前/塔2向后 每组水块间距 (m)
CHASSIS_V      = [0.10, 0.10, math.pi / 3]  # move_for 速度上限 [前后, 横向, 转角]

# ----- 水塔等级标签 → 需搬水块数 -----
WATER_LABEL = {"water_l1": 1, "water_l2": 2, "water_l3": 3}
TARGET_WATER = "water"     # 视觉伺服/对齐用的目标类(水塔/水块)

# ----- 大臂角度 (本仓库物理角度, 由 4_car 度数按场景映射) -----
ARM_SHELF = +93   # 4_car +90: 朝置物架侧 (抓块)
ARM_TOWER = -93   # 4_car -96: 朝水塔侧 (识别/投放)

# ----- 姿态 (mm → m /1000) -----
DETECT_Y     = -0.020                        # 识别水塔时的 y (检测高度)
DETECT_POSE  = dict(x=-0.200, y=DETECT_Y, arm=ARM_TOWER, hand=-70.0)  # 识别水塔姿势 (编排层预摆, 任务只校验)
PICK_POSE_Y  = -0.180                        # 抓块姿态 y (servo_y)
PICK_HAND    = -20                            # 抓块姿态手爪 (-10 )
FIRST_CUBE_X, SECOND_CUBE_X = -0.145, -0.210 # 每块组内 第1/第2 块 X
GRASP_Y, LIFT_Y = -0.065, -0.150             # 吸块下降 y / 吸完抬回 y
DELIVER_Y    = [-0.020, -0.060, -0.090]      # 放块第1/2/3层 y (梯度)
DELIVER_HAND = [-85, -90, -90]               # 放块第1/2/3层手爪 (4_car -80/-85/-85 )
CARRY_X      = [[-0.070, -0.065, -0.065],
                [-0.070, -0.065, -0.065]]    # 每塔每块放块 X (m)

# ----- 转大臂前 XY 安全位 (4_car 安全区 X∈[-300,-200] Y∈[-200,-90]mm; Y 用 -0.18 更高) -----
SAFE_X, SAFE_Y = -0.200, -0.180

# ----- 视觉伺服参数 (现场校准) -----
#   kp=(左右增益, 前后增益); sign=(左右符号, 前后符号)。
#   TRACK: kp_x=0=横向锁死, kp_y=0.22=只前后; sign_y=+1=前后方向(目标左→前进; 现场定, 反则正反馈越追越偏)
TRACK = dict(                                   # 底盘对齐水塔 (只前后)
    cx=0.060, cy=-0.460, kp=(0.0, 0.30),
    sign=(1.0, 1.0), deadband=0.02, hold=6,
    v_max=0.11, timeout=15.0,
)
PICK = dict(                                    # 机械臂伺服抓水块(期望点)
    cx=-0.045, cy=-0.545,                       # 大臂-1/滑轨-1 (跟 seeding 抓取一致)
)
# ----- 分段伺服参数(粗对齐→精对齐, 照抄 seeding._align_staged 规则) -----
# 粗对齐: 大增益(0.70/0.50)快速把水块拉近 + 大死区(0.10), 4s 超时, settle 4 帧即粗到位,
#         lock 5 帧首次锁定。
# 精对齐: 小增益(0.15/0.15)小死区(0.02)精确收敛, 6s 超时, settle 4 帧,
#         lock=1 不重新累计锁定帧, lock_px 锁粗对齐目标(不重新选)。
PICK_COARSE = dict(
    gains=(0.70, 0.50), sign=(-1.0, -1.0), deadzone=0.10, timeout=4.0,
    settle=4, lock=5, debug=True,
)
PICK_FINE = dict(
    gains=(0.15, 0.15), sign=(-1.0, -1.0), deadzone=0.02, timeout=6.0,
    settle=4, lock=1, debug=True,
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
    """切机械臂姿态: 转大臂前 XY 先到安全位(防低Y/伸X时转臂撞塔) → 转大臂/手爪 → XY 并发到位.
    大臂角度不变(无旋转)时跳过安全位机动, 直接 XY 到位——省掉塔间切检测姿态那趟
    重复的抬Y/收X(如放完块 大臂已在 -93, 切检测姿态仍 -93, 抬到 -0.15 纯属无用功).
    末端命令异步发(不等应答), 减少对半双工总线的占用."""
    if car.arm.angle != arm:          # 有旋转才需要"抬Y收X到安全位"防撞
        car.arm.move_y_position(SAFE_Y)
        car.arm.move_x_position(SAFE_X)
    car.arm.set_arm_angle(arm)
    if hand is not None:
        car.arm.set_hand_angle_async(hand, speed=80)
    car.arm.goto_position(x, y)


def _check_detect_pose(car):
    """校验机械臂已到识别水塔姿势 (上一任务结束钉姿势即为识别姿势); 不在则按安全顺序兜底摆回.
    大臂/末端角度是"上次下发值"(舵机无位置回读), 只报状态不阻塞."""
    ax = abs(car.arm.x_pose_now - DETECT_POSE["x"]) < 0.01
    ay = abs(car.arm.y_pose_now - DETECT_POSE["y"]) < 0.01
    aa = abs(car.arm.angle - DETECT_POSE["arm"]) < 2.0
    if ax and ay and aa:
        print(f"[机械臂] 已处于识别水塔姿势 (x={DETECT_POSE['x']}, y={DETECT_POSE['y']}, "
              f"大臂={DETECT_POSE['arm']}, 末端={car.arm.hand_angle})")
        return
    print(f"[机械臂] 不在识别水塔姿势 (x={car.arm.x_pose_now:.3f}, y={car.arm.y_pose_now:.3f}, "
          f"大臂={car.arm.angle}, 末端={car.arm.hand_angle}), 按安全顺序兜底摆回")
    _arm_to(car, DETECT_POSE["x"], DETECT_POSE["y"], DETECT_POSE["arm"], DETECT_POSE["hand"])


# 底盘对齐/识别都算"水"的标签: water(塔/水块) 或 water_l*(等级标), 哪个可见用哪个
WATER_ALIGN_LABELS = ("water", "water_l1", "water_l2", "water_l3")


def _find_water_label(car, max_age=0.5):
    """取实时缓存里任一可见的水标签作为对齐目标(water 或 water_l* 都算)."""
    for d in car.get_realtime_detections(max_age=max_age):
        if d[2] in WATER_ALIGN_LABELS:
            return d[2]
    return None


def _align_tower(car):
    """底盘视觉对齐水塔: 目标取实时缓存里任一可见水标签(先等 1.8s 让其出现),
    只前后(横向锁死),"""
    end = time.time() + 1.8
    label = None
    while time.time() < end:
        label = _find_water_label(car)
        if label is not None:
            break
        time.sleep(0.02)
    if label is None:
        print("[底盘] 1.8s 内未见任何水标签, 跳过对齐")
        return
    print(f"[底盘] 对齐目标: {label}")
    car.chassis_align(label, cx=TRACK["cx"], cy=TRACK["cy"],
                      kp=TRACK["kp"],             # sign 不传 → 按大臂档位自动定(竖拍+1)
                      deadband=TRACK["deadband"], hold=TRACK["hold"],
                      v_max=TRACK["v_max"], decouple_xy=False,
                      timeout=TRACK["timeout"])


def _detect_water_num(car, timeout=0.2):
    """从侧视实时缓存识别水塔等级标 water_l*, 返回需搬块数(没识别到返回 0)."""
    end = time.time() + timeout
    while time.time() < end:
        for d in car.get_realtime_detections(max_age=0.5):
            if d[2] in WATER_LABEL:
                return WATER_LABEL[d[2]], d[2]
        time.sleep(0.01)
    return 0, None


def _ensure_hand(car, target, retries=3, settle=0.2):
    """末端 PWM 舵机无位置回读(只能发不能读), 以连发命令+等舵机到位时间+重试,
    覆盖丢帧/大电流复位, 确保末端确实在 target 角度再继续。
    用异步发(不等应答)降低对半双工总线的占用, 命令送达率更高。
    (现场: 只发一次时常停在半路约 -40 未到位)"""
    for _ in range(retries):
        car.arm.set_hand_angle_async(target, speed=80)
        time.sleep(settle)


def _align_staged(car, label, cx, cy, coarse, fine, prefer_left=False,
                  prefer_right=False, max_px=None, lock_px=None,
                  final_rule=None, px_range=None, min_score=0.0):
    """分段视觉对齐: 粗对齐快拉近(大增益/大死区) → 精对齐精确收敛.
    (照抄 seeding._align_staged)

    精对齐"追踪粗对齐的目标, 不重新选/不重新累计锁定帧":
      - 任务层已传 lock_px 则沿用;
      - 否则取粗对齐后最接近期望点的目标 px 作精对齐 lock_px,
        精对齐每帧强制锁定它(lock=1 不累计, 立即续追)。
    粗对齐超时也照进精对齐(完赛优先)。返回精对齐收敛状态。
    """
    ok_coarse = car.arm_servo_align(
        label, cx, cy, prefer_left=prefer_left, prefer_right=prefer_right,
        max_px=max_px, lock_px=lock_px, final_rule=final_rule,
        px_range=px_range, min_score=min_score, **coarse
    )
    # 精对齐锁定目标: 任务层已锁则沿用; 否则取粗对齐后最接近期望点的目标
    fine_lock = lock_px
    if fine_lock is None:
        dets = [d for d in car.get_realtime_detections(max_age=0.3)
                if d[2] == label and d[3] >= min_score]
        if px_range is not None:
            dets = [d for d in dets if px_range[0] <= d[4] <= px_range[1]]
        if dets:
            fine_lock = min(
                dets, key=lambda d: (abs(d[4] - cx) ** 2 + abs(d[5] - cy) ** 2)
            )[4]
        else:
            fine_lock = cx
    ok_fine = car.arm_servo_align(
        label, cx, cy, prefer_left=prefer_left, prefer_right=prefer_right,
        max_px=max_px, lock_px=fine_lock, final_rule=final_rule,
        px_range=px_range, min_score=min_score, **fine
    )
    return ok_fine or ok_coarse


def _servo_pick(car):
    """车不动, 机械臂视觉伺服把水块对齐到 setpoint → 下探吸 → 抬回."""
    _ensure_hand(car, PICK_HAND)          # 对齐前: 末端强制到位(抓块姿 )
    _align_staged(car, TARGET_WATER, cx=PICK["cx"], cy=PICK["cy"],
                  coarse=PICK_COARSE, fine=PICK_FINE)
    _ensure_hand(car, -15.0)                 # 下降前: 末端转朝下 0
    car.arm.move_y_position(GRASP_Y)      # 降到水块高度
    car.arm.grasp(True)                   # 吸气吸住
    car.arm.move_y_position(LIFT_Y)       # 抬回运输高度


def run_one_tower(car, tower_idx, is_last_tower=False):
    """处理一座水塔: 识别水量 → 逐块「抓→放」. 底盘相对塔原点, 串行.

    安全顺序(每次转大臂前 XY 必在安全位 x=-0.2 / y≥-0.15):
      抓: 抬Y到SAFE_Y(-0.18) → 转大臂+93/末端 → X到组内指定距离 → 伺服识别水块 → 下探吸 → 抬回-0.15
      放: X回SAFE_X(-0.2) → 摆大臂-93+末端(同时) → X/Y到投放位 → 放气
      块间: X回-0.2 → Y回-0.18; 塔末: X回-0.2 → Y回检测姿势(-0.02) 供前进下一塔
    """
    # 机械臂此时已在识别水塔姿势 (run() 已校验/兜底), 直接识别+对齐
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
        car.arm.move_y_position(SAFE_Y)                # 抬Y到安全高 (转大臂前)
        pick_x = FIRST_CUBE_X if k % 2 == 0 else SECOND_CUBE_X
        _arm_to(car, pick_x, PICK_POSE_Y, ARM_SHELF, PICK_HAND)  # 转大臂+93/末端, X→指定距离
        _servo_pick(car)                                 # 伺服识别水块 + 抓取
        # ===== 放 =====
        _chassis(car, pos, 0.0)                          # 底盘回塔
        car.arm.move_y_position(LIFT_Y)                  # 抓完抬回运输高(-0.15)
        # 末端慢, 紧接 Y 抬升后立刻异步转: 与 X/Y 同步跑, 不等 X 移动完再转
        car.arm.set_hand_angle_async(DELIVER_HAND[k], speed=80)
        car.arm.move_x_position(SAFE_X)                  # X回安全位
        car.arm.set_arm_angle(ARM_TOWER)                 # 摆大臂-93
        car.arm.goto_position(CARRY_X[tower_idx][k], DELIVER_Y[k])  # X/Y到投放位
        _ensure_hand(car, DELIVER_HAND[k])               # 投放前: 末端强制到位
        car.arm.grasp(False)                             # 放气投放
        car.beep()
        # ===== 块间/塔末 回位 =====
        car.arm.move_x_position(SAFE_X)                  # 放完先收X到安全位
        if k < n - 1:
            car.arm.move_y_position(SAFE_Y)              # 还有块: 抬回块间安全高
        elif not is_last_tower:
            car.arm.move_y_position(DETECT_Y)            # 塔1末: 回检测姿势, 供前进下一塔
        # 最后一塔末: 只收X到安全位, 交还编排


def run(car):
    """run.py 编排入口(复用编排器已建的车): 编排层已预摆臂到识别姿势, 这里只校验."""
    try:
        _check_detect_pose(car)              # 校验已在识别姿势; 不在则安全兜底摆回

        print("===== 第 1 个水塔 =====")
        run_one_tower(car, tower_idx=0, is_last_tower=False)

        # 塔间前进 (机械臂已在检测姿势: 塔1末块收完X/Y回-0.02, 无需再摆)
        print(f"\n===== 前进 {TOWER_SPACING} m 到第 2 个水塔 =====")
        car.move_for([TOWER_SPACING, 0, 0], max_velocities=CHASSIS_V)

        print("===== 第 2 个水塔 =====")
        run_one_tower(car, tower_idx=1, is_last_tower=True)

        print("\n全部完成 ✓")
        car.beep(); car.beep()
    except KeyboardInterrupt:
        print("\n急停")
    finally:
        car.arm.grasp(False)
        car.stop()


def main():
    """独立运行入口: 自建车后跑 run(). 运行: python tasks/watering.py"""
    car = create_car()
    try:
        run(car)
    finally:
        car.close()


if __name__ == "__main__":
    main()
