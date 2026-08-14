import time


def run(car):
    water_num = {"water_l1": 1, "water_l2": 2, "water_l3": 3}  # 标签对应水量
    tower_water = []
    water_loction = []
    tower_loction = {}
    car.arm.set_arm_pose(x=0.0, y=0.02, arm="RIGHT", hand="UP")

    car.lane_dis_offset(speed=0.3, dis_hold=2.0)
    car.get_odometry(True)
    time.sleep(1)
    car.move_for([0, -0.05, 0])  # 向右微调位置
    # 识别第一个水塔
    cls_id, label = car.move_to_detection_target(delta_y=None)
    tower_water.append(label)
    print(f"识别到目标{cls_id}-{label},第一个水塔")
    car.beep()
    tower_loction[label] = car.get_odometry(True)
    headinng = tower_loction[label][2]
    print(f"当前角度{headinng}")

    # 记录水块位置
    car.arm.move_y_position(0.2)
    # car.arm.move_x_position(0.0)
    car.arm.set_arm_pose(arm="LEFT", hand="DOWN")

    def record_detection_pose():
        """返回识别位置"""
        time.sleep(1)
        cls_id, label = car.move_to_detection_target()
        x, y, z = car.get_odometry()
        pose = [x, y, z, car.arm.x_get_position()]
        car.beep()
        return pose

    # 记录前两个水块位置
    water_loction.append(record_detection_pose())
    car.adjust_arm_position(0.1)
    water_loction.append(record_detection_pose())

    # 记录中间两个水块位置
    car.lane_dis_offset(speed=0.3, dis_hold=0.32)
    water_loction.append(record_detection_pose())
    car.adjust_arm_position(-0.1)
    water_loction.append(record_detection_pose())

    # 记录后两个水块位置
    car.lane_dis_offset(speed=0.3, dis_hold=0.32)
    x, y, z = car.get_odometry()
    car.move_for([0, -0.03, headinng - z])  # 调整角度 不要巡线导致位置歪了
    water_loction.append(record_detection_pose())
    car.adjust_arm_position(0.1)
    water_loction.append(record_detection_pose())

    # 调整位置识别第二个水塔
    car.arm.set_arm_pose(arm="RIGHT", hand="UP")
    car.arm.set_arm_pose(x=0.0, y=0.02)

    time.sleep(0.5)
    cls_id, label = car.move_to_detection_target(delta_y=None)
    tower_water.append(label)
    print(f"识别到目标{cls_id}-{label},第二个水塔")
    car.beep()
    tower_loction[label] = car.get_odometry(True)

    print("------------------水塔任务记录------------------")
    print(f"水塔识别结果：{tower_water}")
    print(f"水块位置：")
    print(*water_loction, sep="\n")
    print(f"水塔位置：{tower_loction}")
    print("----------------------------------------------")
    print("-------------------开始执行--------------------")
    # 先执行第二个水塔，
    for i, label in enumerate(reversed(tower_water)):
        water_num_ = water_num[label]
        print(f"当前水塔{label}，需要浇水{water_num_}次")
        for j in range(water_num_):
            # 移动到水块位置
            car.arm.move_y_position(0.2)
            car.arm.move_x_position(0.0)
            car.arm.set_arm_pose(arm="LEFT", hand="DOWN")
            # 调整位置和机械臂，与水块对齐
            if i == 0:
                k = -(j + 1)
            if i == 1:
                k = j
            car.move_to_position(water_loction[k][0:3])
            car.arm.move_x_position(water_loction[k][3])
            car.move_to_detection_target()
            car.adjust_arm_position()
            # 吸水
            car.arm.grasp(True)
            car.arm.move_y_position(0.09)
            car.arm.move_y_position(0.2)
            car.arm.move_x_position(0.01)
            car.arm.set_arm_pose(arm="RIGHT", hand="UP")

            # 移动到水塔位置
            car.move_to_position(tower_loction[label])
            car.arm.move_y_position(0.01 + 0.055 * j)
            car.move_to_detection_target(delta_y=None)
            car.arm.move_x_position(0.20)
            car.arm.grasp(False)
            time.sleep(0.5)
            car.arm.move_x_position(0.15)
            time.sleep(0.5)
            car.arm.move_x_position(0.01)
            # 浇水
            time.sleep(0.5)
