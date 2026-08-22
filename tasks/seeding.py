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
GRASP_Y, LIFT_Y = -0.00, -0.15  # 降至最底(0)吸 / 抬回(-0.15)
GRASP_HOLD = 0.4  # 吸气位置停驻时长(秒), 吸稳再抬
PLACE_Y, PLACE_LIFT_Y = -0.025, -0.15  # 放苗微降 / 释放后一步抬到 -0.15

MOVE_V = 0.1  # 底盘平移限速, 降漂移

# ── 进入任务点后中线对位(巡线居中 → 原路直退) ─────────────────────
# 巡线前进 3s×0.2m/s ≈ 0.6m 把车拉到道路中间(correction 拉中线), 再原路直退
# 0.6m 回到入口但已居中。回退用 move_for 相对位移(无世界坐标/无横向/无转向漂移)。
CENTER_FWD_SPEED = 0.25  # 巡线前进速度 (m/s)
CENTER_FWD_TIME = 2.5   # 巡线前进时长 (s) ≈ 0.6m
CENTER_FWD_DIS = 0.0    # 原路直退距离 (m) = 3s × 0.2m/s

# ── 摆姿势新轴序(大臂先于 X) ───────────────────────────────────
# set_arm_pose 是 XY 并行→大臂; 播种两处过渡需要"大臂先摆、X 后动":
#   ① 预对位/放苗后→抓取姿势: 大臂从槽位侧摆回抓取侧, 再动 X(防横伸扫过槽位)
#   ② 抓取后→放苗姿势: 先抬 Y, 再摆大臂到放苗侧, 再动 X(防带苗扫过育苗架)
# 大臂舵机无位置回读, 保留 set_arm_pose 的双发兜底 + 到位等待。
ARM_SWING_WAIT = 0.8  # 大臂摆到位等待(秒), 现场可调

# ── 底盘纵向粗调: 目标画面 cx 与预期差过大 → 车前后微调再对齐 ─────────
FINE_TUNE_THRESHOLD = 0.4   # cx 偏差超此值认为底盘没到位(停靠点太前/太后)
FINE_TUNE_STEP = 0.09       # 单次前后微调距离 (m)
FINE_TUNE_V = 0.10          # 微调速度 (m/s)
FINE_TUNE_MAX = 2           # 最多微调次数


# ── 分段伺服参数(粗对齐→精对齐, 抓/放分开, 来自现场标定) ─────────────
# sign 按现场最终确认双表全反(目标在左→该摆向RIGHT、目标在上→该左伸/右缩),
# 现用值: PICK(-1,1), PLACE(-1,-1)。
# 粗对齐: 大增益(0.65/0.65)快速把目标拉近 + 大死区(0.08)容忍误差, 5s 超时,
#         settle=3 帧即算粗到位, lock=5 帧首次锁定。
# 精对齐: 小增益(0.35/0.25)小死区(0.02)精确收敛, 5s 超时, settle=4 帧,
#         lock=1 不重新累计锁定帧, 直接续追粗对齐的目标(lock_px, 不重新选)。
PICK_COARSE = dict(
    gains=(0.65, 0.65), sign=(-1.0, 1.0), deadzone=0.08, timeout=5.0,
    settle=3, lock=5, debug=True,
)
PICK_FINE = dict(
    gains=(0.35, 0.25), sign=(-1.0, 1.0), deadzone=0.02, timeout=5.0,
    settle=4, lock=1, debug=True,
)
PLACE_COARSE = dict(
    gains=(0.65, 0.65), sign=(-1.0, -1.0), deadzone=0.08, timeout=5.0,
    settle=3, lock=5, debug=True,
)
PLACE_FINE = dict(
    gains=(0.35, 0.25), sign=(-1.0, -1.0), deadzone=0.02, timeout=5.0,
    settle=4, lock=1, debug=True,
)

# 画面出现多个目标时的最终决策(传给 arm_servo_align 的 final_rule):
#   所有候选都先锁定(频闪/低置信度不丢), 再按下面规则选最终目标。
#   'right'=靠右 / 'left'=靠左 / 'near'=靠近期望点; None=不启用(用 prefer 原逻辑)
PICK_FINAL_RULE = None

# ── 置信度过滤: 只认 90% 以上的目标, 滤掉低分误检(选列/对齐统一用) ──
SCORE_THRESHOLD = 0.50

# ── 选列扫描参数(抗频闪, 仅用于选列决策; 抓取对齐仍用 SCORE_THRESHOLD) ────
# cylinder 频闪有两种: ① 低分瞬时跌破阈值(用放宽阈值解决)
#                   ② 纯频闪 — 检测出现又消失, 分数可能很高(用跨调用记忆解决)
SCAN_WINDOW = 0.40      # 窗口时长(秒)
SCAN_POLL = 0.05        # 轮询间隔(秒)
SCAN_MIN_COUNT = 2      # 窗口内至少出现 N 次才算在场
SCAN_PERSIST_CY3 = 0.8  # cylinder_3 跨调用记忆窗口(秒): 频闪间隔 > 单窗口也能保留

# 逐 label 扫描阈值(cylinder_3 抗频闪加强, 其他保持一致)
LABEL_SCAN_THRESHOLD = {
    "cylinder_1": 0.30,
    "cylinder_2": 0.30,
    "cylinder_3": 0.20,  # cylinder_3 更激进: 频闪低分也算在场
}

# 模块级: cylinder_3 跨调用最近见到时间(纯频闪场景, 跨 _scan_label 调用保留)
_scan_seen_cy3 = 0.0

# ── 本列画面范围(抗"看到下一列"): 侧视相机能看到左右相邻列 ──────────
# 检测坐标 nx∈[-1,1], 0=画面中心, 负=左, 正=右(见 trt_infer.tolist_nomoralize)。
# 车停靠列位后本列 cylinder 在画面中心附近(各尺寸 NOZZLE cx ∈ ±0.08), 相邻列在左右。
# 选列扫描与抓取对齐都只认 |nx|<=此值 的本列目标, 从根上杜绝抓到相邻列。
# 现场用 view_cam 看相邻列目标 nx 大致在哪再标定此值(相邻列 nx 通常远超 0.35)。
COLUMN_CX_MARGIN = 0.35

# ── 任务白名单: 播种只认这四类标签, 其余一律过滤, 不干扰决策 ──────────
TASK_LABELS = ("cylinder_set", "cylinder_1", "cylinder_2", "cylinder_3")


def _dets(car, max_age=0.3):
    """侧视实时缓存, 只保留白名单四类且置信度达标的检测框(其余标签一律过滤)."""
    return [d for d in car.get_realtime_detections(max_age=max_age)
            if d[2] in TASK_LABELS and d[3] >= SCORE_THRESHOLD]


def _in_column(d, margin=COLUMN_CX_MARGIN):
    """是否本列目标: 画面横坐标 |nx| <= margin(侧视归一化, 0=画面中心).
    相邻列目标 nx 通常在 ±0.4 之外, 由此排除, 避免选列/抓取看到下一列。"""
    return abs(d[4]) <= margin


def _scan_label(car):
    """窗口扫描 + cylinder_3 跨调用持久记忆, 抗 cylinder_3 纯频闪.

    纯频闪场景: 目标瞬时出现/消失(分数可能很高), 单窗口扫描也会漏。
    解法(双保险):
      1) cylinder_3 单独使用更激进阈值 LABEL_SCAN_THRESHOLD["cylinder_3"]=0.20,
         比其他 label 的 0.30 更低, 接受更低分的命中。
      2) 模块级 _scan_seen_cy3 记忆最近见到时间, 在 SCAN_PERSIST_CY3 秒内
         出现过即算在场 — 频闪间隔大于单窗口 SCAN_WINDOW 也能保留。
    其他 label 仍按窗口内累计 >= SCAN_MIN_COUNT 判定。

    在场 labels 中取"出现次数最多"者(并列按 CYLINDERS 顺序); 全无则 None。
    抓取对齐仍走 _dets(SCORE_THRESHOLD=0.50), 此处不抬高低分误检门槛。
    """
    global _scan_seen_cy3

    counts = {l: 0 for l in CYLINDERS}
    end = time.time() + SCAN_WINDOW
    while time.time() < end:
        for d in car.get_realtime_detections(max_age=SCAN_POLL * 2):
            label = d[2]
            if not _in_column(d):          # 只认本列, 排除左右相邻列
                continue
            thr = LABEL_SCAN_THRESHOLD.get(label, 0.30)
            if label in counts and d[3] >= thr:
                counts[label] += 1
                if label == "cylinder_3":
                    _scan_seen_cy3 = time.time()
        time.sleep(SCAN_POLL)

    now = time.time()
    cy3_persisted = (now - _scan_seen_cy3) < SCAN_PERSIST_CY3

    present = []
    for l in CYLINDERS:
        if counts[l] >= SCAN_MIN_COUNT:
            present.append(l)
        elif l == "cylinder_3" and cy3_persisted:
            # cylinder_3 专属: 即使窗口内累计不够, 跨调用记忆里仍有也算在场
            present.append(l)

    if not present:
        return None
    return max(present, key=lambda l: (counts[l], -CYLINDERS.index(l)))


def _ensure_hand(car, target=-20.0, retries=2, settle=0.1):
    """视觉对齐前强制末端手爪到位: 舵机无位置回读(只能发不能读),
    故以"连发命令+等舵机到位时间+重试"覆盖丢帧/大电流复位场景,
    保证手爪确实在 target 角度再开始对齐/抓取。"""
    for _ in range(retries):
        car.arm.set_hand_angle(target)
        time.sleep(settle)


def _arm_pose_seq(car, x=None, y=None, arm=None, hand=None, lift_y_first=False):
    """摆姿势(新轴序): [抬Y→] 转大臂 → 动X → 末端(异步).

    取代 set_arm_pose(XY并行→大臂): 大臂先摆、X 后动, 避免带苗扫过育苗架 /
    机械臂在错误朝向时横伸。大臂保留双发兜底(总线竞争, 同 set_arm_pose) +
    ARM_SWING_WAIT 等舵机摆到位后再动 X。lift_y_first=True 时先 move_y_position(y)。
    """
    if lift_y_first and y is not None:
        car.arm.move_y_position(y)          # ① 先抬Y
    if arm is not None:
        car.arm.set_arm_angle(arm)          # ② 转大臂
        time.sleep(0.3)
        car.arm.set_arm_angle(arm)          #    双发兜底(防总线竞争丢帧)
        if ARM_SWING_WAIT > 0:
            time.sleep(ARM_SWING_WAIT)      #    等大臂真正摆到位再动X
    if x is not None:
        car.arm.move_x_position(x)          # ③ 再动X
    if hand is not None:
        car.arm.set_hand_angle_async(hand)  # 末端异步, 不阻塞


def _align_staged(car, label, cx, cy, coarse, fine, prefer_left=False,
                  prefer_right=False, max_px=None, lock_px=None,
                  final_rule=None, px_range=None, min_score=SCORE_THRESHOLD):
    """分段视觉对齐: 粗对齐快拉近(大增益/大死区/5s) → 精对齐精确收敛.

    精对齐"追踪粗对齐的目标, 不重新选/不重新累计锁定帧":
      - 任务层已传 lock_px(如 cylinder_2 锁最左真目标)则沿用;
      - 否则取粗对齐结束后本列最接近期望点的目标 px 作精对齐 lock_px,
        精对齐每帧强制锁定它(lock=1 不累计, 立即续追)。
    粗对齐超时也照进精对齐(完赛优先)。返回精对齐收敛状态(粗对齐收敛时
    精对齐通常也收敛; 粗对齐超时但精对齐到位也算成功)。
    """
    ok_coarse = car.arm_servo_align(
        label, cx, cy, prefer_left=prefer_left, prefer_right=prefer_right,
        max_px=max_px, lock_px=lock_px, final_rule=final_rule,
        px_range=px_range, min_score=min_score, **coarse
    )
    # 精对齐锁定目标: 任务层已锁则沿用; 否则取粗对齐后本列最接近期望点的目标
    fine_lock = lock_px
    if fine_lock is None:
        dets = [d for d in _dets(car) if d[2] == label]
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
        cy2 = [d for d in _dets(car) if d[2] == "cylinder_2" and _in_column(d)]
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
    # 右臂对齐 cylinder_1/2/3(抓取): 分段粗→精对齐; px_range 只认本列, 排除相邻列
    ok = _align_staged(
        car, label, *NOZZLE[label], coarse=PICK_COARSE, fine=PICK_FINE,
        prefer_right=True, max_px=max_px, lock_px=lock_px,
        final_rule=PICK_FINAL_RULE,
        px_range=(-COLUMN_CX_MARGIN, COLUMN_CX_MARGIN),
    )
    if not ok:
        print(f"[抓取] {label} 对齐未收敛, 继续按当前位下放抓取")
    _ensure_hand(car, -15)  # ② 抓取前再兜底: 视觉对齐后, 末端调 -15 下探(仅此一次改 -15, 其余保持 -20)
    car.arm.grasp(True)                  # 下降前先吸气(下降途中吸嘴已在吸)
    car.arm.move_y_position(GRASP_Y)     # 降到最底(0)吸
    time.sleep(GRASP_HOLD)               # 到底停 0.4s, 吸稳再抬
    car.arm.move_y_position(LIFT_Y)
    return True


def _pick_or_fallback(car, label, completed):
    """_pick 失败时(仅 cylinder_2 全空底座会失败)兜底改抓 cylinder_1/3.

    场景: 扫描本列时 cylinder_2 命中, 但抓取前发现所有 cylinder_2 都是空底座
    (px>0.4), 而本列其实有 cylinder_1/3 可抓 → 不应直接跳过本列。
    返回成功抓取的 label(或 None = 全部失败, 调用方跳过本列)。
    """
    if _pick(car, label):
        return label
    if label != "cylinder_2":
        return None
    # cylinder_2 全空底座 → 兜底改抓其他 label
    alt = next((l for l in ("cylinder_1", "cylinder_3") if l not in completed), None)
    if alt is not None and _pick(car, alt):
        print(f"[播种] cylinder_2 全空底座, 兜底改抓 {alt}")
        return alt
    return None


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
      1) 底盘对齐: 滑轨放 CHASSIS_ALIGN_X(-0.26) 外侧姿态, move_to_detection_target
         车前后 PID + 滑轨跟随, 把 cylinder_set 槽标记移到吸嘴 setpoint(失败/超时也继续)
      2) 机械臂对齐: 把滑轨摆回 PLACE_POSE(-0.2) 基准, 大臂+滑轨把吸嘴
         精确送到槽标记正上方(arm_servo_align)
    放苗的横向/大臂姿势全程通用(槽的横向位置不随筒尺寸变), 所以只做一次,
    后两列放苗直接复用。返回 (x, y, arm, hand) 四自由度姿态。
    预对位超时用 PLACE_POSE 默认值兜底, 不阻塞(完赛优先)。
    """
    # ① 底盘对齐姿态: 滑轨放外侧, 视野敞开便于检测槽标记
    car.arm.set_arm_pose(
        CHASSIS_ALIGN_X, PLACE_POSE["y"], PLACE_POSE["arm"], PLACE_POSE["hand"]
    )
    # 手爪: 大臂摆位+滑轨连发瞬间首条 hand 命令易被总线竞争吞掉/未到位
    # (实测底盘对齐阶段手爪停在 -90)。等臂稳后重发向下并给舵机到位时间。
    time.sleep(0.5)
    car.arm.set_hand_angle(PLACE_POSE["hand"])
    time.sleep(0.5)
    # ── 底盘对齐(老牌 move_to_detection_target 替换 chassis_align): 车前后 PID +
    #    滑轨跟随, 期望点 delta_x/delta_y 取吸嘴 setpoint(MARKER_NOZZLE)。
    #    符号按 arm.side 定(播种放苗侧=LEFT: 目标右→车前进, kp 正号), 先确保;
    #    多目标锁最左(sort_pos=(-2,0), 同原 prefer_left), lock 续追不换目标。
    #    超时/未对齐也继续预对位(完赛优先)。 ──
    car.arm.switch_side("LEFT")
    _ret = car.move_to_detection_target(
        delta_x=MARKER_NOZZLE[0],
        delta_y=MARKER_NOZZLE[1],
        label=MARKER,
        time_out=10.0,
        sort_pos=(-2, 0),
        lock=True,
        min_score=SCORE_THRESHOLD,
    )
    # 底盘对齐轮系高频占总线, 手爪可能又被挤回, 补发一次
    car.arm.set_hand_angle(PLACE_POSE["hand"])
    print(f"底盘对齐槽标记: {_ret}")
    # ② 机械臂预对位: 车已粗对准, 把滑轨摆回 -0.2 基准再让臂精对位
    car.arm.set_arm_pose(
        PLACE_POSE["x"], PLACE_POSE["y"], PLACE_POSE["arm"], PLACE_POSE["hand"]
    )
    _ensure_hand(car)  # 视觉对齐前: 强制末端到位
    # 左臂对齐 cylinder_set(放苗预对位): 分段粗→精对齐, 锁定画面靠左的目标
    ok = _align_staged(
        car, MARKER, *MARKER_NOZZLE, coarse=PLACE_COARSE, fine=PLACE_FINE,
        prefer_left=True,
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
    global _scan_seen_cy3
    _scan_seen_cy3 = 0.0  # 清空上一轮残留记忆, 防跨任务误选
    pos = [0.0]  # 底盘纵向自记账
    seen = None
    completed = []
    place_pose = None  # 第1列预对位记住的放苗姿态, 后两列复用
    # 进入任务点后定位: 先前进 0.4m → 左转 45°(相对当前朝向, 无世界坐标/无横移)
    car.move_for([0.4, 0.0, 0.0], max_velocities=[0.2, 0.1, 0.1])
    car.move_for([0.0, 0.0, math.pi / 4], max_velocities=[0.1, 0.1, math.pi / 6])
    # 进入任务点后中线对位: 巡线前进居中(correction 拉中线) → 原路直退。
    # lane 恒速取 v_forward; 低速用温和转向 PID(全局默认 kp 太大, 0.2m/s 会蛇形)。
    print(
        f"\n===== 中线对位: 巡线前进 {CENTER_FWD_TIME}s ({CENTER_FWD_SPEED}m/s) ====="
    )
    with car.lane_config(
        dict(
            kp=1.0,  # 低速专用转向 PID
            v_forward=CENTER_FWD_SPEED,  # 真按 0.2m/s 恒速
            corr_threshold=0.05,  # 中线叠加防抖阈值抬高
            corr_weight=0.2,  # 中线叠加强度减弱
        )
    ):
        car.lane_time(speed=CENTER_FWD_SPEED, time_dur=CENTER_FWD_TIME)
    print(f"===== 原路直退 {CENTER_FWD_DIS}m =====")
    car.move_for([-CENTER_FWD_DIS, 0, 0], max_velocities=[MOVE_V, MOVE_V, math.pi / 3])
    for col in (1, 2, 3):
        _chassis(car, SOURCE[col], pos)
        # 第1列: 先预对位槽标记, 记住放苗姿势(横向/大臂姿势全程通用, 只需一次)
        if place_pose is None:
            place_pose = _pre_align(car, pos)
        # 摆抓取姿势(新轴序: 先转大臂再动X; y 已是 -0.15 无需动)
        _arm_pose_seq(
            car, PICK_POSE["x"], PICK_POSE["y"], PICK_POSE["arm"], PICK_POSE["hand"]
        )
        print(f"已经移动到了PICK_POSE")
        # 扫描本列 cylinder; 没有就用剩余 label 兜底
        label = _scan_label(car)
        if label is None:
            label = next((l for l in CYLINDERS if l not in completed), None)
            if label is None:
                continue
        # 1↔3 纠错(同尺寸易认错)
        if seen is None:
            seen = label
        elif label == seen and seen in ("cylinder_1", "cylinder_3"):
            label = "cylinder_3" if seen == "cylinder_1" else "cylinder_1"
        picked = _pick_or_fallback(car, label, completed)
        if picked is None:
            print(f"[播种] 列{col} 全部候选未抓取, 跳过本列(不放苗)")
            continue
        label = picked
        completed.append(label)
        # 放苗: 底盘到槽列 + 直接用第1列记住的放苗姿势(不再现场伺服)
        _chassis(car, SLOT[TARGET_SLOT[label]], pos)
        px, py, parm, phand = place_pose
        # 放苗(新轴序: 先抬Y → 转大臂 → 动X)
        _arm_pose_seq(car, px, py, parm, phand, lift_y_first=True)
        _place(car)
    # 全部搬完 → 回到 s3 (最后一个抓取位置, SOURCE[3]=0.30m)
    print(f"[播种] 全部搬完, 回到 s3 (SOURCE[3]=0.30m)")
    _chassis(car, SOURCE[3], pos)
    return completed
