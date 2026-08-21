import time

from tasks.target_detection import ANIMAL_CONF

LANE_PID = dict(
    kp=0.0, ki=0.0, kd=0.0,     # 转向 PID: 全局 6.5 -> 4.0, 防直道摆动
    limits=(0, 0),         # 角速度限幅: 全局 ±4.5 -> ±3.0
    deadzone=0.05,              # da 死区: 直线噪声 ~0.1, 全局 0.0 未启用 -> 削掉
)

X_C_LIST = [0.36, 0.27, 0.31, 0.29]

# 对齐窗口(宽): move_to_detection_target 只对齐 x_c 距目标站位在该范围内的 animal,
# 需罩住入场位置, 偏窄会导致找不到候选而对齐空等超时。现场按摄像头视野标定。
ALIGN_RANGE = 0.8
# 击倒判定窗口(窄): 站位 x_c 附近找不到 animal 才算击倒, 避免误判远处残骸。
KNOCK_RANGE = 0.17

ANIMAL_CONF = 0.7

MAX_SHOTS = 5  # 每个击倒点最多击发次数, 击倒即停

FINAL_ADVANCE_M = 0.5  # 击倒 2 个后收尾前进的固定距离(现场标定)

def _knocked_down(car, x_c, range_=KNOCK_RANGE):
    """击倒判定: 站位 x_c 附近找不到 animal ⇒ 已击倒."""
    dets = [
        d for d in car.get_detection_results(score_thresh=ANIMAL_CONF)
        if d[2] == "animal" and abs(d[4] - x_c) <= range_
    ]
    return not dets

def _chassis(car, target, pos):
    """闭环行驶到绝对位置 target(m), 自记账, 不依赖 odom 绝对值."""
    dx = target - pos[0]
    if abs(dx) < 0.05:
        return
    # 车道保持行驶(与 lane_dis_offset 一致), 每次只发相对量 dx
    car.lane_dis_offset(speed=0.10, dis_hold=dx)
    pos[0] = target

def run(car, animal_list=None):  # noqa: E741

    if animal_list is None:
        animal_list = [1] * 4  # 外部未传时默认, x_c 用 X_C_LIST
    # 害/益 保留外部 value, x_c 一律用自己指定的 X_C_LIST (缺省兜底 0.44)
    animal_list = [
        (v, X_C_LIST[i]) if i < len(X_C_LIST) else (v, 0.34)
        for i, (v, _) in enumerate((x, 0.0) if isinstance(x, int) else x for x in animal_list)
    ]
    print("animal_list =", animal_list)
    step = 0.15  # 每个目标间距
    hit_idx = []  # 打击点所在 index, 绝对站位 = idx*step
    hit_x = []  # 对应击打点动物的 x_c (target_detection 记录, 用于 sort_pos 选中该只)
    for idx, item in enumerate(animal_list):
        value, x_c = item  # animal_list[i] = (害/益, x_c)
        if value == 0:  # 遇到需要打击的点
            hit_idx.append(idx)
            hit_x.append(x_c)  # 记录该击打点动物的 x_c

    # 射击任务
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.25, y=-0.04)
    with car.lane_config(LANE_PID):
        car.move_to_detection_target(
                delta_x=x_c, delta_y=None,label="animal",sort_pos=(0, 0),
                min_score=ANIMAL_CONF)  # 对齐 animal_list[0] (用它的 x_c 选中)
        knock_count = 0  # 已击倒数
        pos = [0.0]  # 底盘纵向自记账, 起点=对齐 animal_list[0] 处
        for idx, x_c in zip(hit_idx, hit_x):
            _chassis(car, idx * step, pos)
            time.sleep(0.2)
            for _ in range(MAX_SHOTS):  # 每点最多击发 MAX_SHOTS 次, 击倒即停
                car.move_to_detection_target(  # 每次击发前重新对齐(未击倒才走到这)
                    delta_x=x_c, delta_y=None, label="animal", sort_pos=(0.17, 0), lock=True,
                    min_score=ANIMAL_CONF, select_range=ALIGN_RANGE)  # 对齐击打点 (用 x_c 选中该只)
                time.sleep(0.2)
                car.beep()
                car.shooting()
                time.sleep(0.3)
                if _knocked_down(car, x_c):
                    knock_count += 1
                    print(f"击倒 {knock_count}/2")
                    break
            time.sleep(0.3)
            if knock_count >= 2:  # 击倒 2 个即整个射击停止
                time.sleep(0.3)
                break