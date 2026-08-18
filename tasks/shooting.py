import time


def run(car, animal_list=None):  # noqa: E741
    if animal_list is None:
        animal_list = [0, 0, 0, 0]

    step = 0.16  # 每个目标间距
    relative_loc = []  # 记录相对运动距离
    last_index = -1  # 记录上一个打击点的索引，初始为-1
    d_x = 0.2  # 对齐参数

    for idx, value in enumerate(animal_list):
        if value == 0:  # 遇到需要打击的点
            if last_index == -1:
                # 第一个打击点：相对距离 = 从起点走到这里
                dist = idx * step
            else:
                # 后续打击点：相对距离 = 两个点之间的间隔数 * 0.16
                dist = (idx - last_index) * step

            relative_loc.append(dist)
            last_index = idx  # 更新上一个打击点位置
    print(relative_loc)

    # 射击任务
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.25, y=-0.04)
    # 对齐第一个目标
    car.move_to_detection_target(delta_x=d_x, delta_y=None, sort_pos=(d_x, 0))

    for dis in relative_loc:
        car.lane_dis_offset(speed=0.2, dis_hold=dis)
        cls_id, label = car.move_to_detection_target(
            delta_x=d_x, delta_y=None, sort_pos=(d_x, 0)
        )
        time.sleep(1)
        car.beep()
        car.shooting()
        time.sleep(1)

    car.lane_dis_offset(
        speed=0.2, dis_hold=0.48 - sum(relative_loc)
    )  # 距离补偿到最后一个目标
