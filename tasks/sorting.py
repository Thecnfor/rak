import time


def run(car):
    ball_list = [0.0, 0.06]  # 拿黄球时 机械臂x轴0.0, 蓝球0.06

    # 调整机械臂
    car.arm.move_y_position(0.17)
    car.arm.move_x_position(0.30)
    car.arm.set_arm_pose(arm="LEFT", hand=-70)
    car.arm.move_y_position(0.05)

    # 移动到任务位置 前进2.0米
    car.lane_dis_offset(speed=0.3, dis_hold=2.0)
    time.sleep(0.5)
    # 对齐到标签
    cls_id, label = car.move_to_detection_target(delta_y=None)
    time.sleep(0.5)
    # 根据标签颜色确定要拿的小球
    if label == "lable_blue":
        flag = 1
    else:
        flag = 0

    for i in range(2):
        for j in range(4):
            # 从储存架拿球
            car.arm.move_y_position(0.15)
            car.arm.set_arm_pose(arm=-107, hand=10)  # 放球位置
            car.arm.move_x_position(ball_list[(i + flag) % 2])  # 移动机械臂x轴
            car.arm.grasp(True)
            car.arm.move_y_position(0.08)
            car.arm.move_y_position(0.15)
            car.arm.move_x_position(0.30)
            car.arm.set_arm_pose(arm=94, hand="UP")
            car.arm.move_y_position(0.2 - i * 0.15)
            time.sleep(0.5)
            car.arm.move_x_position(0.2)
            car.arm.grasp(False)
            time.sleep(0.5)
            car.arm.move_x_position(0.30)
        if i == 1:
            break
        car.move_for([-0.155, 0, 0])


# 寻找货物的程序
