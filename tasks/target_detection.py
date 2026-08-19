import time

LANE_PID = dict(
    kp=0.0, ki=0.0, kd=0.0,     # 转向 PID: 全局 6.5 -> 4.0, 防直道摆动
    limits=(0, 0),         # 角速度限幅: 全局 ±4.5 -> ±3.0
    deadzone=0.05,              # da 死区: 直线噪声 ~0.1, 全局 0.0 未启用 -> 削掉
)

# 动物识别置信度阈值: 置信度大于此值的 animal 才标记进 animal_list
ANIMAL_CONF = 0.85

def run(car) -> list:
    # 每元素 (害/益, x_c): 害=0 需击打, 益=1; x_c 为该站位动物的归一化横坐标
    animal_list = [(0, 0.0)] * 4
    car._lane_v_min = 0.20
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.2, y=-0.05)
    car.get_distance(True)
    time.sleep(0.4)

    with car.lane_config(LANE_PID):
        for i in range(4):
            cls_id, label = car.move_to_detection_target(
                delta_x=0.0, delta_y=None, sort_pos=(0,0),min_score=ANIMAL_CONF
            )
            time.sleep(0.2)
            if label == "animal":
                # 置信度过滤: 只有高分 animal 才写入; 取距 x_c=0 最近那只(当前站位对准的)记它的 x_c
                dets = car.get_detection_results()
                high = [d for d in dets if d[2] == "animal" and d[3] > ANIMAL_CONF]
                if high:
                    det = min(high, key=lambda d: abs(d[4]))
                    res, analysis = car.animal_image_analysis()
                    if res is not None:
                        car.beep()
                        print(f"第{i+1}个动物分析结果：{res}，{analysis}")
                        animal_list[i] = (res, det[4])
            time.sleep(0.2)
            car.move_distance([0.2, 0, 0], dis=0.14)
        time.sleep(0.2)
        car.beep()
        car.get_distance(True)
        return animal_list
