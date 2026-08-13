from cv2 import sort
from smartcar import logger, CountRecord
from car_wrap_2026 import MyCar, kill_other_python
import time
import math

"""
- cylinder_3
- h_jin_zhen_gu
- h_tu_dou
- h_xi_lan_hua
- h_dou_jiao
- h_you_cai
- animal
- h_qin_cai
- cylinder_2
- cylinder_1
- cylinder_set
- h_qing_jiao
- h_fan_qie
- water_l3
- water_l2
- water
- ball_blue
- ball_yellow
- storage
- lable_yellow
- order
- h_mo_gu
- name
- lable_blue
"""


def init():
    time.sleep(1)
    global my_car
    my_car = MyCar()
    my_car.STOP_PARAM = False
    my_car.beep()
    time.sleep(1)
    my_car.arm.reset_position()
    my_car.reset_position()  #

def auto_lane_tracing(speed=0.3, dis_hold=0.85):
    my_car.lane_dis_offset(speed=speed, dis_hold=dis_hold)
    print(f"巡线停止的位置：{my_car.get_odometry()}")

def auto_seeding():

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
    my_car.arm.set_arm_pose(0.0, 0.2, "LEFT", "DOWN")
    my_car.lane_dis_offset(speed=0.3, dis_hold=0.85)
    time.sleep(0.5)
    print(f"巡线停止的位置：{my_car.get_odometry()}")

    for i in range(3):
        my_car.move_to_position(cylinder_loc[cylinder_list[i]])
        my_car.move_to_detection_target()
        x, y, z = my_car.get_odometry()
        pose = [x, y, z, my_car.arm.x_get_position()]
        print(f"第{i}个播种位置{pose}")
        cylinder_set_list[cylinder_list[i]] = pose
        my_car.beep()
    print("实际播种位置：")
    print(cylinder_set_list)

    for i in range(3):
        # 移动手臂到右侧高处
        my_car.arm.move_y_position(0.2)
        my_car.arm.move_x_position(0.3)
        my_car.arm.set_arm_pose(arm="RIGHT")

        # 对齐目标，识别
        my_car.move_to_position(cylinder_loc[cylinder_list[i]])
        time.sleep(0.5)
        cls_id, label = my_car.move_to_detection_target()
        print(f"识别到目标{cls_id}-{label}")
        my_car.beep()
        pose = cylinder_set_list[label]

        # 调整气泵吸嘴对齐目标
        my_car.adjust_arm_position()
        # 吸起目标
        my_car.arm.grasp(True)
        my_car.arm.move_y_position(0.01)
        time.sleep(0.5)
        my_car.arm.move_y_position(0.2)

        # 移动到目标播种处
        my_car.arm.move_x_position(pose[3])
        my_car.arm.set_arm_pose(arm="LEFT")
        time.sleep(1)
        my_car.move_to_position(pose[:3])
        my_car.adjust_arm_position()
        my_car.arm.move_y_position(0.04)
        my_car.arm.grasp(False)
        time.sleep(1)

    my_car.arm.move_y_position(0.1)
    my_car.arm.set_arm_pose(hand="UP")
    my_car.arm.move_x_position(0.15)
    my_car.move_to_position(cylinder_loc[cylinder_list[0]])
    print("播种完成")
    my_car.beep()
    my_car.beep()
    my_car.get_odometry(True)
    my_car.get_distance(True)


def target_shooting_detection() -> list:

    animal_list = [0, 0, 0, 0]
    my_car.arm.set_arm_pose(x=0.05, y=0.05, arm="LEFT", hand="UP")
    my_car.lane_dis_offset(speed=0.3, dis_hold=1.45)

    _x, _y, _z = my_car.get_odometry(True)
    my_car.get_distance(True)
    my_car.move_for([0, 0, 0 - _z])
    time.sleep(3)

    for i in range(4):
        my_car.lane_dis_offset(speed=0.3, dis_hold=0.15)
        time.sleep(0.5)
        cls_id, label = my_car.move_to_detection_target(delta_y=None)
        if label == "animal":
            res, analysis = my_car.animal_image_analysis()
            if res is not None:
                my_car.beep()
                print(f"第{i}个动物分析结果：{res}，{analysis}")
                animal_list[i] = res
    time.sleep(0.5)
    my_car.beep()
    my_car.beep()
    my_car.get_odometry(True)
    my_car.get_distance(True)
    return animal_list


def water_tower_task():
    water_num = {"water_l1": 1,"water_l2": 2, "water_l3": 3}  # 标签对应水量
    tower_water = []
    water_loction = []
    tower_loction = {}
    my_car.arm.set_arm_pose(x=0.0, y=0.02, arm="RIGHT", hand="UP")

    my_car.lane_dis_offset(speed=0.3, dis_hold= 2.0)
    my_car.get_odometry(True)
    time.sleep(1)
    my_car.move_for([0, -0.05, 0])  # 向右微调位置
    # 识别第一个水塔
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    tower_water.append(label)
    print(f"识别到目标{cls_id}-{label},第一个水塔")
    my_car.beep()
    tower_loction[label] = my_car.get_odometry(True)
    headinng = tower_loction[label][2]
    print(f"当前角度{headinng}")

    # 记录水块位置
    my_car.arm.move_y_position(0.2)
    # my_car.arm.move_x_position(0.0)
    my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")

    def record_detection_pose():
        """返回识别位置"""
        time.sleep(1)
        cls_id, label = my_car.move_to_detection_target()
        x, y, z = my_car.get_odometry()
        pose = [x, y, z, my_car.arm.x_get_position()]
        my_car.beep()
        return pose

    # 记录前两个水块位置
    water_loction.append(record_detection_pose())
    my_car.adjust_arm_position(0.1)
    water_loction.append(record_detection_pose())

    # 记录中间两个水块位置
    my_car.lane_dis_offset(speed=0.3, dis_hold=0.32)
    water_loction.append(record_detection_pose())
    my_car.adjust_arm_position(-0.1)
    water_loction.append(record_detection_pose())

    # 记录后两个水块位置
    my_car.lane_dis_offset(speed=0.3, dis_hold=0.32)
    x, y, z = my_car.get_odometry()
    my_car.move_for([0, -0.03, headinng - z])  # 调整角度 不要巡线导致位置歪了
    water_loction.append(record_detection_pose())
    my_car.adjust_arm_position(0.1)
    water_loction.append(record_detection_pose())

    # 调整位置识别第二个水塔
    my_car.arm.set_arm_pose(arm="RIGHT", hand="UP")
    my_car.arm.set_arm_pose(x=0.0, y=0.02)

    time.sleep(0.5)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    tower_water.append(label)
    print(f"识别到目标{cls_id}-{label},第二个水塔")
    my_car.beep()
    tower_loction[label] = my_car.get_odometry(True)

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
            my_car.arm.move_y_position(0.2)
            my_car.arm.move_x_position(0.0)
            my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")
            # 调整位置和机械臂，与水块对齐
            if i == 0:
                k = -(j + 1)
            if i == 1:
                k = j
            my_car.move_to_position(water_loction[k][0:3])
            my_car.arm.move_x_position(water_loction[k][3])
            my_car.move_to_detection_target()
            my_car.adjust_arm_position()
            # 吸水
            my_car.arm.grasp(True)
            my_car.arm.move_y_position(0.09)
            my_car.arm.move_y_position(0.2)
            my_car.arm.move_x_position(0.01)
            my_car.arm.set_arm_pose(arm="RIGHT", hand="UP")

            # 移动到水塔位置
            my_car.move_to_position(tower_loction[label])
            my_car.arm.move_y_position(0.01 + 0.055 * j)
            my_car.move_to_detection_target(delta_y=None)
            my_car.arm.move_x_position(0.20)
            my_car.arm.grasp(False)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.15)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.01)
            # 浇水
            time.sleep(0.5)


def target_shooting(animal_list=[0, 0, 0, 0]):  # noqa: E741

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
    my_car.arm.set_arm_pose(arm="LEFT", hand="UP")
    my_car.arm.set_arm_pose(x=0.3, y=0.02)

    my_car.lane_dis_offset(speed=0.3, dis_hold=3.0)
    my_car.move_for([-0.2, 0, 0])
    # 对齐第一个目标
    my_car.move_to_detection_target(delta_x=d_x, delta_y=None, sort_pos=(d_x, 0))

    for dis in relative_loc:
        my_car.lane_dis_offset(speed=0.3, dis_hold=dis)
        cls_id, label = my_car.move_to_detection_target(
            delta_x=d_x, delta_y=None, sort_pos=(d_x, 0)
        )
        time.sleep(5)
        my_car.beep()
        my_car.shooting()
        time.sleep(5)

    my_car.lane_dis_offset(
        speed=0.3, dis_hold=0.48 - sum(relative_loc)
    )  # 距离补偿到最后一个目标


def crop_harvesting():
    """
    作物采收
    """
    # 调整机械臂
    my_car.arm.move_y_position(0.2)
    my_car.arm.reset_x()
    my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")

    my_car.set_storage(True)  # 抬起存储架

    # 移动到任务位置
    my_car.lane_dis_offset(speed=0.3, dis_hold=2.3)
    my_car.arm.move_y_position(0.17)

    for i in range(8):
        # 调整机械臂
        my_car.arm.move_x_position(0.0)
        my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")
        # 前进一小段
        my_car.lane_dis_offset(speed=0.3, dis_hold=0.04)
        time.sleep(0.5)
        # 对齐目标
        cls_id, label = my_car.move_to_detection_target(delta_x=-0.05, time_out=3.0)
        print(f"发现第{i + 1}个作物，目标为{label}")
        time.sleep(0.5)
        my_car.adjust_arm_position()
        my_car.arm.grasp(True)
        time.sleep(0.3)
        my_car.arm.move_y_position(0.045)  # 吸取
        time.sleep(0.3)
        my_car.arm.move_y_position(0.17)  # 抬起机械臂
        time.sleep(0.3)
        my_car.arm.set_arm_pose(arm=-115, hand=10)  # 放球位置
        if label == "ball_yellow":  # 黄球在一号位
            my_car.arm.move_x_position(0.0)
            my_car.beep()
        elif label == "ball_blue":
            my_car.arm.move_x_position(0.06)
            my_car.beep()
            my_car.beep()
        time.sleep(1)
        my_car.arm.grasp(False)
        time.sleep(1)
    my_car.set_storage(False)  # 放下存储架


def sort_and_store():
    ball_list = [0.0, 0.06]  # 拿黄球时 机械臂x轴0.0, 蓝球0.06

    # 调整机械臂
    my_car.arm.move_y_position(0.17)
    my_car.arm.move_x_position(0.30)
    my_car.arm.set_arm_pose(arm="LEFT", hand=-70)
    my_car.arm.move_y_position(0.05)

    # 移动到任务位置 前进2.0米
    my_car.lane_dis_offset(speed=0.3, dis_hold=2.0)
    time.sleep(0.5)
    # 对齐到标签
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    time.sleep(0.5)
    # 根据标签颜色确定要拿的小球
    if label == "lable_blue":
        flag = 1
    else:
        flag = 0

    for i in range(2):
        for j in range(4):
            # 从储存架拿球
            my_car.arm.move_y_position(0.15)
            my_car.arm.set_arm_pose(arm=-107, hand=10)  # 放球位置
            my_car.arm.move_x_position(ball_list[(i + flag) % 2])  # 移动机械臂x轴
            my_car.arm.grasp(True)
            my_car.arm.move_y_position(0.08)
            my_car.arm.move_y_position(0.15)
            my_car.arm.move_x_position(0.30)
            my_car.arm.set_arm_pose(arm=94, hand="UP")
            my_car.arm.move_y_position(0.2 - i * 0.15)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.2)
            my_car.arm.grasp(False)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.30)
        if i == 1:
            break
        my_car.move_for([-0.155, 0, 0])


# 寻找货物的程序
def find_goods(label, dy=-0.5):
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    my_car.arm.move_x_position(0.20)
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    my_car.move_for([0.15, 0, 0])
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    my_car.arm.move_x_position(0.30)
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label


def get_order():
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

    my_car.arm.reset_position()
    my_car.lane_dis_offset(speed=0.3, dis_hold=1.5)
    # 对齐订单
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    # 推动推杆
    my_car.move_for([0.065, 0, 0])
    my_car.arm.move_x_position(0.23)
    my_car.arm.move_x_position(0.1, out_time=4.0)
    time.sleep(0.5)
    # 识别随机标签
    my_car.move_for([-0.06, 0, 0])
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    time.sleep(0.5)
    text_list.append(my_car.get_ocr(label="order"))
    my_car.beep()
    # 识别固定标签
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.21)
    my_car.arm.set_hand_angle("MID")
    my_car.arm.set_arm_angle("RIGHT")
    time.sleep(0.5)
    cls_id, label = my_car.move_to_detection_target()
    time.sleep(1)
    my_car.get_detection_results()
    text_list.append(my_car.get_ocr(label="order"))
    my_car.beep()

    print(text_list)
    # 使用大模型分析订单
    for text in text_list:
        if text is None:
            order_list.append(None)
            continue
        order_info = my_car.order_analysis.get_res_json(text)
        order_list.append(order_info)
    # 对订单排序，先拿2号楼的
    order_list.sort(key=lambda x: x["address"])
    print(order_list)

    my_car.lane_dis_offset(speed=0.3, dis_hold=0.2)
    my_car.arm.set_hand_angle(angle="DOWN")

    loc = my_car.get_odometry(True)

    my_car.set_storage(True)  # 抬起存储架
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.30)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    goods_now = order_list[1]["goods"]
    find_goods(goods_dict[goods_now])
    print(f"正在拿取第一个货物：{goods_now}")
    time.sleep(0.5)
    my_car.arm.grasp(True)
    my_car.arm.move_y_position(0.05)
    time.sleep(0.5)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.0)
    my_car.arm.move_y_position(0.09)
    time.sleep(0.5)
    my_car.arm.grasp(False)
    # 拿第二个货物
    my_car.move_to_position(loc)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.30)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    goods_now = order_list[0]["goods"]
    find_goods(goods_dict[goods_now])
    print(f"正在拿取第二个货物：{goods_now}")
    time.sleep(0.5)
    my_car.arm.grasp(True)
    my_car.arm.move_y_position(0.05)
    time.sleep(0.5)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.0)
    my_car.arm.move_y_position(0.14)
    time.sleep(0.5)
    my_car.arm.grasp(False)

    my_car.move_to_position(loc)
    return order_list


def find_name(name="name"):
    name_list = []
    for i in range(3):
        my_car.move_to_detection_target(delta_y=None)
        time.sleep(1)
        dets = my_car.get_detection_results(sort_pos=(0, 0.5), limit_x=0.3)
        for j, det in enumerate(dets):
            text = my_car.get_det_ocr(det)
            print(f'第{i}列第{j}行的姓名：{text}')
            time.sleep(5)
            if text == name:
                return i, j  # i为0 是下层，为上层
        if i < 2:
            my_car.lane_dis_offset(speed=0.3, dis_hold=0.11)


def order_delivery(    order_list = [
        {"name": "李四", "goods": "芹菜", "address": 2},
        {"name": "钱七", "goods": "青椒", "address": 2},
    ]):


    my_car.lane_dis_offset(speed=0.3, dis_hold=3.25)

    time.sleep(1)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.3)
    my_car.arm.set_arm_pose(arm="LEFT", hand=-70)
    time.sleep(1)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    if label is None:
        my_car.lane_dis_offset(speed=0.3, dis_hold=0.12)
    time.sleep(1)
    # 记录1号楼起始位置
    loc_flag = 1
    loc = my_car.get_odometry(True)

    for i, order in enumerate(order_list):
        my_car.move_to_position(loc)
        if order["address"] > loc_flag:
            my_car.lane_dis_offset(speed=0.3, dis_hold=0.56)
            loc_flag = 2
            loc = my_car.get_odometry(True)
        time.sleep(0.5)

        # 调节识别高度
        my_car.arm.move_y_position(0.13)
        my_car.arm.move_x_position(0.3)
        my_car.arm.set_arm_pose(arm="LEFT", hand='UP')

        _x, y = find_name(order["name"])
        my_car.arm.set_arm_pose(arm="RIGHT", hand="DOWN")
        my_car.arm.move_x_position(0.0)
        my_car.arm.grasp(True)
        my_car.arm.move_y_position(0.135 - i * 0.05)
        my_car.arm.move_y_position(0.155 - i * 0.05)
        my_car.arm.move_x_position(0.2)
        my_car.arm.set_arm_pose(arm="LEFT", hand=-70)
        my_car.arm.move_y_position(y * 0.09)
        my_car.arm.move_x_position(0.1)
        my_car.arm.grasp(False)
        time.sleep(1)
        my_car.arm.move_x_position(0.15)
        my_car.arm.set_arm_pose(arm="LEFT", hand=-80)
        time.sleep(0.5)
        my_car.arm.move_x_position(0.2)
    
    if loc_flag == 1:
        my_car.lane_dis_offset(speed=0.3, dis_hold=1.7)
    else:
        my_car.lane_dis_offset(speed=0.3, dis_hold=1.1)
