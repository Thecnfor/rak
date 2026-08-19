import time

from tasks.target_detection import ANIMAL_CONF

LANE_PID = dict(
    kp=0.0, ki=0.0, kd=0.0,     # 转向 PID: 全局 6.5 -> 4.0, 防直道摆动
    limits=(0, 0),         # 角速度限幅: 全局 ±4.5 -> ±3.0
    deadzone=0.05,              # da 死区: 直线噪声 ~0.1, 全局 0.0 未启用 -> 削掉
)

X_C_LIST = [0.40, 0.40, 0.40, 0.40]

def run(car, animal_list=None):

    if animal_list is None:
        animal_list = [(1, 0.0)] * 4  # 外部未传时默认, x_c 用 X_C_LIST
    # 害/益 保留外部 value, x_c 一律用自己指定的 X_C_LIST (缺省兜底 0.44)
    animal_list = [
        (v, X_C_LIST[i]) if i < len(X_C_LIST) else (v, 0.44)
        for i, (v, _) in enumerate((x, 0.0) if isinstance(x, int) else x for x in animal_list)
    ]
    print("animal_list =", animal_list)
    step = 0.18  # 每个目标间距
    relative_loc = []  # 记录相对运动距离
    hit_x = []  # 对应击打点动物的 x_c (target_detection 记录, 用于 sort_pos 选中该只)
    last_index = -1  # 记录上一个打击点的索引，初始为-1
    d_x = 0.68  # 对齐参数

    for idx, item in enumerate(animal_list):
        value, x_c = item  # animal_list[i] = (害/益, x_c)
        
        if value == 0:  # 遇到需要打击的点
            if last_index == -1:
                # 第一个打击
                
                # 后续打击点：相对距离 = 两个点之间的间隔数 * step
                dist = (idx - last_index) * step

            relative_loc.append(dist)
            hit_x.append(x_c)  # 记录该击打点动物的 x_c
            last_index = idx  # 更新上一个打击点位置

    # 射击任务
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.25, y=-0.04)
    with car.lane_config(LANE_PID):
        car.move_to_detection_target(
                delta_x=d_x, delta_y=None,label="animal",sort_pos=(animal_list[0][1],0),
                min_score=ANIMAL_CONF)  # 对齐 animal_list[0] (用它的 x_c 选中)
        for dis, x_c in zip(relative_loc, hit_x):
            if dis > 0:
                car.lane_dis_offset(speed=0.15, dis_hold=dis)
                time.sleep(1)
            car.move_to_detection_target(
                            delta_x=d_x, delta_y=None,label="animal",sort_pos=(x_c,0),lock=True,
                            min_score=ANIMAL_CONF)  # 对齐击打点 (用 x_c 选中该只)
            time.sleep(0.5)
            car.beep()
            car.shooting()
            time.sleep(0.5)