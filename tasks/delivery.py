import time

def find_name(car, name="name"):
    name_list = []
    for i in range(3):
        car.move_to_detection_target(delta_y=None)
        time.sleep(1)
        dets = car.get_detection_results(sort_pos=(0, 0.5), limit_x=0.3)
        for j, det in enumerate(dets):
            text = car.get_det_ocr(det)
            print(f'第{i}列第{j}行的姓名：{text}')
            time.sleep(5)
            if text == name:
                return i, j  # i为0 是下层，为上层
        if i < 2:
            car.lane_dis_offset(speed=0.3, dis_hold=0.11)


def run(car, order_list=None):
    if order_list is None:
        order_list = [
        {"name": "李四", "goods": "芹菜", "address": 2},
        {"name": "钱七", "goods": "青椒", "address": 2},
        ]


    car.lane_dis_offset(speed=0.3, dis_hold=3.25)

    time.sleep(1)
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(0.3)
    car.arm.set_arm_pose(arm="LEFT", hand=-70)
    time.sleep(1)
    cls_id, label = car.move_to_detection_target(delta_y=None)
    if label is None:
        car.lane_dis_offset(speed=0.3, dis_hold=0.12)
    time.sleep(1)
    # 记录1号楼起始位置
    loc_flag = 1
    loc = car.get_odometry(True)

    for i, order in enumerate(order_list):
        car.move_to_position(loc)
        if order["address"] > loc_flag:
            car.lane_dis_offset(speed=0.3, dis_hold=0.56)
            loc_flag = 2
            loc = car.get_odometry(True)
        time.sleep(0.5)

        # 调节识别高度
        car.arm.move_y_position(0.13)
        car.arm.move_x_position(0.3)
        car.arm.set_arm_pose(arm="LEFT", hand='UP')

        _x, y = find_name(car, order["name"])
        car.arm.set_arm_pose(arm="RIGHT", hand="DOWN")
        car.arm.move_x_position(0.0)
        car.arm.grasp(True)
        car.arm.move_y_position(0.135 - i * 0.05)
        car.arm.move_y_position(0.155 - i * 0.05)
        car.arm.move_x_position(0.2)
        car.arm.set_arm_pose(arm="LEFT", hand=-70)
        car.arm.move_y_position(y * 0.09)
        car.arm.move_x_position(0.1)
        car.arm.grasp(False)
        time.sleep(1)
        car.arm.move_x_position(0.15)
        car.arm.set_arm_pose(arm="LEFT", hand=-80)
        time.sleep(0.5)
        car.arm.move_x_position(0.2)
    
    if loc_flag == 1:
        car.lane_dis_offset(speed=0.3, dis_hold=1.7)
    else:
        car.lane_dis_offset(speed=0.3, dis_hold=1.1)
