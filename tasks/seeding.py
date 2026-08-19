#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务1: 自动移苗(播种) — 三列育苗筒 -> 种植槽.

视觉伺服已提为通用方法 car.arm_servo_align(tasks/tools/motion/locate.py),
本文件只保留任务编排与抓/放动作; 其余底盘/姿态/吸放均映射现有 SDK。
参数未测试以及运动逻辑和方向需要测试
"""
import math
import time

# ── 底盘列(相对位移 m) & 标签→槽映射 ──────────────────────────────
SOURCE = {1: 0.0, 2: 0.15, 3: 0.30}
SLOT = {1: 0.0, 2: 0.15, 3: 0.30}
TARGET_SLOT = {
    "cylinder_1": 3,
    "cylinder_2": 2,
    "cylinder_3": 1,
}  # 3大→槽1, 2中→槽2, 1小→槽3(左→右由大到小)
CYLINDERS = ("cylinder_1", "cylinder_2", "cylinder_3")

# ── 吸嘴 setpoint(目标在吸嘴正下方时 bbox 中心, 归一化) — 需重标 ──
NOZZLE = {
    "cylinder_1": (0.080, -0.310),
    "cylinder_2": (-0.010, -0.294),
    "cylinder_3": (-0.009, -0.396),
}
MARKER = "cylinder_set"
MARKER_NOZZLE = (-0.000, -0.230)

# ── 姿态(角度直读, 不用字符串; x/y 米) — 需重标 ────────────────────
#    Y 轴方向: 向下为正, 0=最底, -0.2=最顶(抬升为负值, 与 arm_motion 标定一致)
#    大臂角度: LEFT=93 / MID=0 / RIGHT=-93; 末端角度: UP=-90 / MID=-37 / DOWN=0
#    注: 末端"竖直向下"角度 2026-08-18 现场定为 -10(PICK/PLACE/_ensure_hand 均用 -10)。
#    尺寸→槽(重要): cylinder_3=最大筒→槽1(最近), cylinder_2=中筒→槽2,
#    cylinder_1=最小筒→槽3(最远); 即槽列位置从近到远 1/2/3 对应 大/中/小。
PICK_POSE = dict(x=-0.03, y=-0.15, arm=-91, hand=-20)
PLACE_POSE = dict(x=-0.21, y=-0.15, arm=93, hand=-20)  # 机械臂预对位基准/放苗兜底
CHASSIS_ALIGN_X = -0.26  # 仅底盘对齐阶段: 滑轨放更外侧, 便于把槽标拉进画面中心
GRASP_Y, LIFT_Y = -0.01, -0.15  # 降至最底(0)吸 / 抬回(-0.15)
PLACE_Y, PLACE_LIFT_Y = -0.02, -0.15  # 放苗微降 / 释放后一步抬到 -0.15

MOVE_V = 0.1  # 底盘平移限速, 降漂移

# ── 底盘纵向粗调: 目标画面 cx 与预期差过大 → 车前后微调再对齐 ─────────
FINE_TUNE_THRESHOLD = 0.4   # cx 偏差超此值认为底盘没到位(停靠点太前/太后)
FINE_TUNE_STEP = 0.05       # 单次前后微调距离 (m)
FINE_TUNE_V = 0.10          # 微调速度 (m/s)
FINE_TUNE_MAX = 2           # 最多微调次数


# ── 伺服参数(抓/放分开, 来自 4_car task_config.yml) ───────────────────
# sign 按现场最终确认双表全反(目标在左→该摆向RIGHT、目标在上→该左伸/右缩),
# 现用值: PICK(-1,1), PLACE(-1,-1), 对齐超时 10s。debug=True 待收敛确认后再删。
PICK_SERVO = dict(
    gains=(0.28, 0.18), sign=(-1.0, 1.0), deadzone=0.01, timeout=15.0, debug=True
)
PLACE_SERVO = dict(
    gains=(0.25, 0.15), sign=(-1.0, -1.0), deadzone=0.01, timeout=15.0, debug=True
)

# 画面出现多个目标时的最终决策(传给 arm_servo_align 的 final_rule):
#   所有候选都先锁定(频闪/低置信度不丢), 再按下面规则选最终目标。
#   'right'=靠右 / 'left'=靠左 / 'near'=靠近期望点; None=不启用(用 prefer 原逻辑)
PICK_FINAL_RULE = None

# ── 置信度过滤: 只认 90% 以上的目标, 滤掉低分误检(选列/对齐统一用) ──
SCORE_THRESHOLD = 0.50

# ── 任务白名单: 播种只认这四类标签, 其余一律过滤, 不干扰决策 ──────────
TASK_LABELS = ("cylinder_set", "cylinder_1", "cylinder_2", "cylinder_3")


def _dets(car, max_age=0.3):
    """侧视实时缓存, 只保留白名单四类且置信度达标的检测框(其余标签一律过滤)."""
    return [d for d in car.get_realtime_detections(max_age=max_age)
            if d[2] in TASK_LABELS and d[3] >= SCORE_THRESHOLD]


def _has(car, label, max_age=0.3):
    """只查不移动: 侧视实时缓存里是否存在该 label 目标(白名单+置信度已过滤, 供选列/兜底扫描用)."""
    return any(d[2] == label for d in _dets(car, max_age))


def _ensure_hand(car, target=-20.0, retries=3, settle=0.5):
    """视觉对齐前强制末端手爪到位: 舵机无位置回读(只能发不能读),
    故以"连发命令+等舵机到位时间+重试"覆盖丢帧/大电流复位场景,
    保证手爪确实在 target 角度再开始对齐/抓取。"""
    for _ in range(retries):
        car.arm.set_hand_angle(target)
        time.sleep(settle)


def _pick(car, label):
    """抓 cylinder_1/2/3: 右臂对齐吸嘴到筒正上方 → 下放吸起.

    对齐超时未收敛也不中断任务: 打印告警后仍按当前臂位继续下放抓取
    (NOZZLE setpoint 已标定, 未收敛多半只是差几个死区, 硬抓成功率更高)。

    注: 底盘前后粗调(advance/backup)只对 cylinder_set 预对位生效,
    抓取 cylinder_1/2/3 不做底盘前后微调(见 _pre_align)。

    cylinder_2 专属规则(空底座误检成 cylinder_2):
      - 画面里 px>0.4 的 cylinder_2 一律视为底座: 不抓取、不锁定(伺服全程由 max_px 剔除)。
      - 剩余真目标里只抓最左边那个(lock_px 锁定, 无视 prefer 靠左/靠右)。
      - 若全部 px>0.4(只剩底座) → 返回 False, 调用方跳过本列不抓不放苗。
    返回 True=已下放抓取 / False=判定为底座、跳过未抓。
    """
    _ensure_hand(car)  # ① 视觉对齐前: 强制末端到位
    max_px = lock_px = None
    if label == "cylinder_2":
        cy2 = [d for d in _dets(car) if d[2] == "cylinder_2"]
        valid = [d for d in cy2 if d[4] <= 0.4]     # px>0.4 = 底座, 剔除
        if not valid:
            print("[抓取] cylinder_2 全部 px>0.4(空底座), 不抓取")
            return False
        valid.sort(key=lambda d: d[4])
        # 未启用 final_rule 时锁最左真目标; 启用时交给 final_rule 多候选决策
        if PICK_FINAL_RULE is None:
            max_px, lock_px = 0.4, valid[0][4]      # 只锁最左边的真目标
        else:
            max_px = 0.4                            # 底座剔除仍生效, 最终决策交给 final_rule
    # 右臂对齐 cylinder_1/2/3(抓取): 锁定画面靠右的目标
    ok = car.arm_servo_align(
        label, *NOZZLE[label], prefer_right=True,
        max_px=max_px, lock_px=lock_px, final_rule=PICK_FINAL_RULE,
        min_score=SCORE_THRESHOLD, **PICK_SERVO
    )
    if not ok:
        print(f"[抓取] {label} 对齐未收敛, 继续按当前位下放抓取")
    _ensure_hand(car)  # ② 抓取前再兜底: 滑轨/Y 大电流移动可能又把舵机打回 -90
    car.arm.move_y_position(GRASP_Y)
    car.arm.grasp(True)
    car.arm.move_y_position(LIFT_Y)
    return True


def _place(car):
    """按调用方已到位的放苗姿势直接落下: 微降放苗 → 释放 → 一步抬离.

    前面 set_arm_pose(预对位记住的姿势)已把臂放到槽正上方, 这里不再现场伺服。
    """
    car.arm.move_y_position(PLACE_Y)
    car.arm.grasp(False)
    car.arm.move_y_position(PLACE_LIFT_Y)


def _pre_align(car, pos=None):
    """第1列预对位: 摆放苗姿势并伺服对齐槽标记, 记住此刻 4 轴放苗姿态.

    两段对齐(顺序固定, 后段不依赖前段成功, 都完赛优先):
      1) 底盘对齐: 滑轨放 CHASSIS_ALIGN_X(-0.25) 外侧姿态, 车前后/左右横移,
         把 cylinder_set 槽标记移到画面中心(失败/超时也继续, 只为粗对准)
      2) 机械臂对齐: 把滑轨摆回 PLACE_POSE(-0.2) 基准, 大臂+滑轨把吸嘴
         精确送到槽标记正上方(arm_servo_align)
    放苗的横向/大臂姿势全程通用(槽的横向位置不随筒尺寸变), 所以只做一次,
    后两列放苗直接复用。返回 (x, y, arm, hand) 四自由度姿态。
    预对位超时用 PLACE_POSE 默认值兜底, 不阻塞(完赛优先)。
    """
    # # ① 底盘对齐姿态: 滑轨放外侧, 视野敞开便于检测槽标记
    # car.arm.set_arm_pose(
    #     CHASSIS_ALIGN_X, PLACE_POSE["y"], PLACE_POSE["arm"], PLACE_POSE["hand"]
    # )
    # # 手爪: 大臂摆位+滑轨连发瞬间首条 hand 命令易被总线竞争吞掉/未到位
    # # (实测底盘对齐阶段手爪停在 -90)。等臂稳后重发向下并给舵机到位时间。
    # time.sleep(0.5)
    # car.arm.set_hand_angle(PLACE_POSE["hand"])
    # time.sleep(0.5)
    # # ── 底盘对齐(新: sign 按大臂档位自动定 sign_y, 横向符号现场探针自证 sign_x,
    # #               kp/deadband/v_min 用 locate 新默认; 镜像 test_chassis_align.py --align) ──
    # # 期望点 cx/cy 取吸嘴 setpoint(MARKER_NOZZLE), decouple_xy=True(默认), 多目标锁最左
    # ok_chassis = car.chassis_align(MARKER, cx=MARKER_NOZZLE[0], cy=MARKER_NOZZLE[1],
    #                                prefer_left=True, probe_sign_x=True,
    #                                timeout=10.0)  # 车动, 槽标记居中; 超时/未对齐也继续预对位
    # # 底盘对齐轮系高频占总线, 手爪可能又被挤回, 补发一次
    # car.arm.set_hand_angle(PLACE_POSE["hand"])
    # print(f"底盘对齐槽标记: {'成功' if ok_chassis else '超时/未对齐, 继续预对位'}")
    # ② 机械臂预对位: 车已粗对准, 把滑轨摆回 -0.2 基准再让臂精对位
    car.arm.set_arm_pose(
        PLACE_POSE["x"], PLACE_POSE["y"], PLACE_POSE["arm"], PLACE_POSE["hand"]
    )
    _ensure_hand(car)  # 视觉对齐前: 强制末端到位
    # 底盘纵向粗调(仅 cylinder_set 适用): 槽标记画面 cx 偏差过大 → 车前后微调再对齐
    if pos is not None:
        _chassis_fine_tune(car, MARKER, MARKER_NOZZLE[0], pos)
    # 左臂对齐 cylinder_set(放苗预对位): 锁定画面靠左的目标
    ok = car.arm_servo_align(
        MARKER, *MARKER_NOZZLE, prefer_left=True, min_score=SCORE_THRESHOLD, **PLACE_SERVO
    )
    if ok:
        # 伺服后臂已微调到槽正上方, 记下此刻 4 轴状态作为放苗姿势
        pose = (
            car.arm.x_get_position(),
            car.arm.y_get_position(),
            car.arm.angle,
            car.arm.hand_angle,
        )
        print(
            f"预对位成功, 记住放苗姿势 x={pose[0]:.3f} y={pose[1]:.3f} "
            f"arm={pose[2]:.1f} hand={pose[3]:.1f}"
        )
        return pose
    print("预对位超时, 放苗用默认姿势")
    return (PLACE_POSE["x"], PLACE_POSE["y"], PLACE_POSE["arm"], PLACE_POSE["hand"])


def _chassis(car, target, pos):
    """闭环 move_for 到相对位移 target(m), 自记账, 不依赖 odom 绝对值."""
    dx = target - pos[0]
    if abs(dx) < 0.05:
        return
    car.move_for([dx, 0.0, 0.0], max_velocities=[MOVE_V, MOVE_V, math.pi / 3])
    pos[0] = target


def _chassis_fine_tune(car, label, expected_cx, pos, max_steps=FINE_TUNE_MAX):
    """底盘纵向粗调: 目标画面 cx 与预期差 > 阈值时, 车前后微调再对齐.

    规则(播种侧视相机): 目标靠画面太右 → 车前进; 靠左 → 车后退。
    画面多个目标以最左那个为准(与偏好锁定的目标一致)。
    微调移动会同步更新 pos 自记账, 后续列定位不受影响。
    仅 cylinder_set(放苗预对位)启用; cylinder_1/2/3 抓取不做底盘前后粗调。
    返回 True=已到位(无需再调) / False=调满 max_steps 仍超差(交给后续对齐兜底)。
    """
    for _ in range(max_steps):
        dets = [d for d in _dets(car) if d[2] == label]
        if not dets:
            return True
        dets.sort(key=lambda d: d[4])
        dev = dets[0][4] - expected_cx
        if abs(dev) < FINE_TUNE_THRESHOLD:
            return True
        dx = FINE_TUNE_STEP if dev > 0 else -FINE_TUNE_STEP
        print(f"[底盘粗调] {label} cx偏差{dev:+.3f}超{FINE_TUNE_THRESHOLD}: "
              f"{'前进' if dx > 0 else '后退'} {FINE_TUNE_STEP:.2f}m")
        car.move_for([dx, 0.0, 0.0], max_velocities=[FINE_TUNE_V, FINE_TUNE_V, math.pi / 3])
        pos[0] += dx
    return False


def run(car):
    pos = [0.0]  # 底盘纵向自记账
    seen = None
    completed = []
    place_pose = None  # 第1列预对位记住的放苗姿态, 后两列复用
    for col in (1, 2, 3):
        _chassis(car, SOURCE[col], pos)
        # 第1列: 先预对位槽标记, 记住放苗姿势(横向/大臂姿势全程通用, 只需一次)
        if place_pose is None:
            place_pose = _pre_align(car, pos)
        # 摆抓取姿势
        car.arm.set_arm_pose(
            PICK_POSE["x"], PICK_POSE["y"], PICK_POSE["arm"], PICK_POSE["hand"]
        )
        print(f"已经移动到了PICK_POSE")
        # 扫描本列 cylinder; 没有就用剩余 label 兜底
        label = next((l for l in CYLINDERS if _has(car, l)), None)
        if label is None:
            label = next((l for l in CYLINDERS if l not in completed), None)
            if label is None:
                continue
        # 1↔3 纠错(同尺寸易认错)
        if seen is None:
            seen = label
        elif label == seen and seen in ("cylinder_1", "cylinder_3"):
            label = "cylinder_3" if seen == "cylinder_1" else "cylinder_1"
        if not _pick(car, label):
            print(f"[播种] 列{col} 判定 {label} 为空底座/未抓取, 跳过本列(不放苗)")
            continue
        completed.append(label)
        # 放苗: 底盘到槽列 + 直接用第1列记住的放苗姿势(不再现场伺服)
        _chassis(car, SLOT[TARGET_SLOT[label]], pos)
        px, py, parm, phand = place_pose
        car.arm.set_arm_pose(px, py, parm, phand)
        _place(car)
    return completed
