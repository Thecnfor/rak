import time

# ----- 巡线转向 PID (本任务专用, 只覆盖 shooting 的对齐推进段) -----
# 全局 cfg 默认 Kp=6.5/死区0.0 太灵敏, 直道蛇形; 这里降增益 + 加死区削直线噪声。
# 由 lane_config 上下文管理: 进段应用、段末(含异常)自动还原, 不污染后续任务。
LANE_PID = dict(
    kp=0.07, ki=0.0, kd=0.0,     # 转向 PID: 全局 6.5 -> 4.0, 防直道摆动
    limits=(-0.5, 0.5),         # 角速度限幅: 全局 ±4.5 -> ±3.0
    deadzone=0.05,              # da 死区: 直线噪声 ~0.1, 全局 0.0 未启用 -> 削掉
)

def run(car, animal_list=None):  # noqa: E741
    if animal_list is None:
        animal_list = [0, 0, 0, 0]

    step = 0.12  # 每个目标间距
    relative_loc = []  # 记录相对运动距离
    last_index = -1  # 记录上一个打击点的索引，初始为-1
    d_x = 0.7  # 对齐参数

    for idx, value in enumerate(animal_list):
        if value == 0:  # 遇到需要打击的点
            if last_index == -1:
                # 第一个打击点：相对距离 = 从起点走到这里
                dist = idx * step
            else:
                # 后续打击点：相对距离 = 两个点之间的间隔数 * 0.12
                dist = (idx - last_index) * step

            relative_loc.append(dist)
            last_index = idx  # 更新上一个打击点位置

    # 射击任务
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.25, y=-0.04)
    # 对齐+推进全程套 LANE_PID(巡线段生效), with 退出即还原, 不污染后续任务
    with car.lane_config(LANE_PID):
        for dis in relative_loc:
            car.lane_dis_offset(speed=0.1, dis_hold=dis)
            cls_id, label = car.move_to_detection_target(
                delta_x=d_x, delta_y=None, sort_pos=(d_x, 0),label="animal")
            time.sleep(0.5)
            car.beep()
            car.shooting()
            time.sleep(0.5)

        car.lane_dis_offset(
            speed=0.15, dis_hold=0.45 - sum(relative_loc)
        )  # 距离补偿到最后一个目标