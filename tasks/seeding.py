import math
import time


def run(car):
    end_dis = car.get_distance() + 0.5
    car.move_base([0.3, 0, 0], lambda: car.get_distance() > end_dis)

    x_length = 0.45  # 基地前方转角的位置，用于计算播种位置
    dis = 0.55  # 转角后第一个播种点的距离
    heading = math.pi / 4  # 车子的方向 45°
    sin45 = math.sin(heading)  # sin45°
    # 正对播种点车子的理论位置
    cylinder_loc = {
        "cylinder_3": [x_length + dis * sin45, dis * sin45, heading],
        "cylinder_2": [x_length + (dis + 0.15) * sin45, (dis + 0.15) * sin45, heading],
        "cylinder_1": [x_length + (dis + 0.3) * sin45, (dis + 0.3) * sin45, heading],
    }
    cylinder_list = ["cylinder_3", "cylinder_2", "cylinder_1"]
    cylinder_set_list = {}

    # 设置机械臂初始状态
    car.arm.set_arm_pose(0.0, 0.2, "LEFT", "DOWN")
    car.lane_dis_offset(speed=0.3, dis_hold=0.85)
    time.sleep(0.5)
    print(f"巡线停止的位置：{car.get_odometry()}")

    for i in range(3):
        car.move_to_position(cylinder_loc[cylinder_list[i]])
        car.move_to_detection_target()
        x, y, z = car.get_odometry()
        pose = [x, y, z, car.arm.x_get_position()]
        print(f"第{i}个播种位置{pose}")
        cylinder_set_list[cylinder_list[i]] = pose
        car.beep()
    print("实际播种位置：")
    print(cylinder_set_list)

    for i in range(3):
        # 移动手臂到右侧高处
        car.arm.move_y_position(0.2)
        car.arm.move_x_position(0.3)
        car.arm.set_arm_pose(arm="RIGHT")

        # 对齐目标，识别
        car.move_to_position(cylinder_loc[cylinder_list[i]])
        time.sleep(0.5)
        cls_id, label = car.move_to_detection_target()
        print(f"识别到目标{cls_id}-{label}")
        car.beep()
        pose = cylinder_set_list[label]

        # 调整气泵吸嘴对齐目标
        car.adjust_arm_position()
        # 吸起目标
        car.arm.grasp(True)
        car.arm.move_y_position(0.01)
        time.sleep(0.5)
        car.arm.move_y_position(0.2)

        # 移动到目标播种处
        car.arm.move_x_position(pose[3])
        car.arm.set_arm_pose(arm="LEFT")
        time.sleep(1)
        car.move_to_position(pose[:3])
        car.adjust_arm_position()
        car.arm.move_y_position(0.04)
        car.arm.grasp(False)
        time.sleep(1)

    car.arm.move_y_position(0.1)
    car.arm.set_arm_pose(hand="UP")
    car.arm.move_x_position(0.15)
    car.move_to_position(cylinder_loc[cylinder_list[0]])
    print("播种完成")
    car.beep()
    car.beep()
    car.get_odometry(True)
    car.get_distance(True)
