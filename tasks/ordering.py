import time


def find_goods(car, label, dy=-0.5):
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    car.arm.move_x_position(0.20)
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    car.move_for([0.15, 0, 0])
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    car.arm.move_x_position(0.30)
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label


def run(car):
    # 标签对应关系
    goods_dict = {
        "青椒": "h_qing_jiao",
        "蘑菇": "h_mo_gu",
        "芹菜": "h_qin_cai",
        "番茄": "h_fan_qie",
        "油菜": "h_you_cai",
        "豆角": "h_dou_jiao",
        "西兰花": "h_xi_lan_hua",
        "土豆": "h_tu_dou",
        "金针菇": "h_jin_zhen_gu",
    }

    text_list = []  # 订单的文本信息
    order_list = []  # 订单的大模型分析信息

    car.arm.reset_position()
    car.lane_dis_offset(speed=0.3, dis_hold=1.5)
    # 对齐订单
    cls_id, label = car.move_to_detection_target(delta_y=None)
    # 推动推杆
    car.move_for([0.065, 0, 0])
    car.arm.move_x_position(0.23)
    car.arm.move_x_position(0.1, out_time=4.0)
    # 识别随机标签
    car.move_for([-0.06, 0, 0])
    cls_id, label = car.move_to_detection_target(delta_y=None)
    time.sleep(0.2)
    text_list.append(car.get_ocr(label="order"))
    car.beep()
    # 识别固定标签
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(0.21)
    car.arm.set_hand_angle("MID")
    car.arm.set_arm_angle("RIGHT")
    time.sleep(0.3)
    cls_id, label = car.move_to_detection_target()
    time.sleep(0.2)
    text_list.append(car.get_ocr(label="order"))
    car.beep()

    print(text_list)
    # 使用大模型分析订单
    for text in text_list:
        if text is None:
            order_list.append(None)
            continue
        order_info = car.order_analysis.get_res_json(text)
        order_list.append(order_info)
    # 对订单排序，先拿2号楼的
    order_list.sort(key=lambda x: x["address"])
    print(order_list)

    car.lane_dis_offset(speed=0.3, dis_hold=0.2)
    car.arm.set_hand_angle(angle="DOWN")

    loc = car.get_odometry(True)

    car.set_storage(True)  # 抬起存储架
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(0.30)
    cls_id, label = car.move_to_detection_target(delta_y=None)
    goods_now = order_list[1]["goods"]
    find_goods(car, goods_dict[goods_now])
    print(f"正在拿取第一个货物：{goods_now}")
    time.sleep(0.2)
    car.arm.grasp(True)
    car.arm.move_y_position(0.05)
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(0.0)
    car.arm.move_y_position(0.09)
    time.sleep(0.2)
    car.arm.grasp(False)
    # 拿第二个货物
    car.move_to_position(loc)
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(0.30)
    cls_id, label = car.move_to_detection_target(delta_y=None)
    goods_now = order_list[0]["goods"]
    find_goods(car, goods_dict[goods_now])
    print(f"正在拿取第二个货物：{goods_now}")
    time.sleep(0.2)
    car.arm.grasp(True)
    car.arm.move_y_position(0.05)
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(0.0)
    car.arm.move_y_position(0.14)
    time.sleep(0.2)
    car.arm.grasp(False)

    car.move_to_position(loc)
    return order_list
