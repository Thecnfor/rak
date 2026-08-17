import time
def run(car) -> list:

    animal_list = [0, 0, 0, 0]
    car._lane_v_min = 0.20
    car.arm.set_arm_pose(x=-0.20, y=0, arm="LEFT", hand="UP")
    car.get_distance(True)
    time.sleep(1)

    for i in range(4):
        cls_id, label = car.move_to_detection_target(delta_x=0.2, delta_y=None, sort_pos=(0.2, 0))
        time.sleep(0.2)
        if label == "animal":
            res, analysis = car.animal_image_analysis()
            if res is not None:
                car.beep()
                print(f"第{i+1}个动物分析结果：{res}，{analysis}")
                animal_list[i] = res
        time.sleep(0.5)
        car.lane_dis_offset(speed=0.2, dis_hold=0.14)
    time.sleep(0.5)
    car.beep()
    car.beep()
    car.get_odometry(True)
    car.get_distance(True)
    return animal_list
