import time

def find_goods(car, label, dy=0):
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    car.arm.move_x_position(-0.12)
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    car.arm.move_x_position(-0.05)
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label
    
    car.move_for([0.07, 0, 0])
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    car.arm.move_x_position(-0.12)
    cls_id, det_label = car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

def _grab_goods(car, goods_dict, goods_name, name, arm_y_final):
    """按订单货物名查找并抓取，任何一步失败都跳过抓取而不是盲抓。"""
    if goods_name not in goods_dict:
        print(f"未识别货物类别: {goods_name}")
        return False
    if not find_goods(car, goods_dict[goods_name]):
        print(f"未找到货物: {goods_name}")
        return False
    print(f"正在拿取{name}:{goods_name}")
    time.sleep(0.2)
    
    car.arm.move_y_position(0.05)
    car.arm.grasp(True)
    car.arm.move_y_position(arm_y_final)
    car.arm.move_x_position(0.0)
    time.sleep(0.2)
    car.arm.grasp(False)
    return True

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
    car.arm.set_arm_pose(x=-0.15, y=0.0)
    car.arm.set_arm_pose(arm="RIGHT", hand="MID")
    # 推动推杆
    car.move_for([0.065, 0, 0])
    car.arm.move_x_position(-0.05)
    car.beep()
    # 识别随机标签
    car.arm.move_x_position(-0.20)
    car.move_for([-0.065, 0, 0])
    car.arm.set_arm_pose(x=-0.12, y=-0.05)
    time.sleep(0.3)
    text_list.append(car.get_ocr(label="order"))
    car.beep()

    # 识别固定标签
    car.arm.move_y_position(-0.20)
    car.arm.move_x_position(-0.06)
    time.sleep(0.2)
    text_list.append(car.get_ocr(label="order"))

    print(text_list)
    # 使用大模型分析订单，跳过解析失败的条目（OCR失败返回None，解析异常返回字符串）
    for text in text_list:
        if text is None:
            continue
        order_info = car.order_analysis.get_res_json(text)
        if not isinstance(order_info, dict) or "goods" not in order_info or "address" not in order_info:
            print(f"订单解析失败: {text}")
            continue
        order_list.append(order_info)
    # 对订单排序，先拿2号楼的
    order_list.sort(key=lambda x: x["address"])
    print(order_list)
    if not order_list:
        print("未识别到有效订单")
        return []

    car.lane_dis_offset(speed=0.2, dis_hold=0.20)
    car.arm.set_hand_angle(angle="DOWN")

    loc = car.get_odometry(True)

    car.set_storage(True)  # 抬起存储架
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(-0.15)
    # 先拿2号楼的货（排序后是索引1）；订单不足2条时跳过
    if len(order_list) >= 2:
        _grab_goods(car, goods_dict, order_list[1]["goods"], "第一个货物", 0.09)
    # 拿第二个货物（1号楼）
    car.move_to_position(loc)
    car.arm.move_y_position(0.2)
    car.arm.move_x_position(-0.15)
    _grab_goods(car, goods_dict, order_list[0]["goods"], "第二个货物", 0.14)

    #抓取完之后移动到指定的地方结束
    car.move_to_position(loc)
    car.move_distance([0.3,0,0],1.3)
    car .set_velocity(0.0,0.0,-1.9)
    return order_list
