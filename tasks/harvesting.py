import time

def run(car):
    """
    作物采收
    """
    # 调整机械臂
    car.arm.move_y_position(0.2)
    car.arm.reset_x()
    car.arm.set_arm_pose(arm="LEFT", hand="DOWN")

    car.set_storage(True)  # 抬起存储架

    # 移动到任务位置
    car.lane_dis_offset(speed=0.3, dis_hold=2.3)
    car.arm.move_y_position(0.17)

    for i in range(8):
        # 调整机械臂
        car.arm.move_x_position(0.0)
        car.arm.set_arm_pose(arm="LEFT", hand="DOWN")
        # 前进一小段
        car.lane_dis_offset(speed=0.3, dis_hold=0.04)
        time.sleep(0.5)
        # 对齐目标
        cls_id, label = car.move_to_detection_target(delta_x=-0.05, time_out=3.0)
        print(f"发现第{i + 1}个作物，目标为{label}")
        time.sleep(0.5)
        car.adjust_arm_position()
        car.arm.grasp(True)
        time.sleep(0.3)
        car.arm.move_y_position(0.045)  # 吸取
        time.sleep(0.3)
        car.arm.move_y_position(0.17)  # 抬起机械臂
        time.sleep(0.3)
        car.arm.set_arm_pose(arm=-115, hand=10)  # 放球位置
        if label == "ball_yellow":  # 黄球在一号位
            car.arm.move_x_position(0.0)
            car.beep()
        elif label == "ball_blue":
            car.arm.move_x_position(0.06)
            car.beep()
            car.beep()
        time.sleep(1)
        car.arm.grasp(False)
        time.sleep(1)
    car.set_storage(False)  # 放下存储架
