import time

from tasks.target_detection import align_forward


def run(car, animal_list=None):  # noqa: E741
    if animal_list is None:
        animal_list = [0, 0, 0, 0]

    step = 0.16  # 每个目标间距
    d_x = 0.2    # 对齐距离

    targets = [i for i, v in enumerate(animal_list) if v == 0]
    relative_loc = [(b - a) * step for a, b in zip([0] + targets, targets)]
    print(relative_loc)

    align = lambda: align_forward(car, delta_x=d_x, delta_y=None, sort_pos=(d_x, 0))  # noqa: E731

    # 射击任务
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.25, y=-0.04)
    align()  # 对齐第一个目标

    for dis in relative_loc:
        car.lane_dis_offset(speed=0.2, dis_hold=dis)
        align()
        time.sleep(1)
        car.beep()
        car.shooting()
        time.sleep(1)

    car.lane_dis_offset(speed=0.2, dis_hold=0.48 - sum(relative_loc))  # 补偿到最后一个目标
