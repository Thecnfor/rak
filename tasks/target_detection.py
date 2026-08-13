import time

def run(car) -> list:

    animal_list = [0, 0, 0, 0]
    car.arm.set_arm_pose(x=0.05, y=0.05, arm="LEFT", hand="UP")
    car.lane_dis_offset(speed=0.3, dis_hold=1.45)

    _x, _y, _z = car.get_odometry(True)
    car.get_distance(True)
    car.move_for([0, 0, 0 - _z])
    time.sleep(3)

    for i in range(4):
        car.lane_dis_offset(speed=0.3, dis_hold=0.15)
        time.sleep(0.5)
        cls_id, label = car.move_to_detection_target(delta_y=None)
        if label == "animal":
            res, analysis = car.animal_image_analysis()
            if res is not None:
                car.beep()
                print(f"第{i}个动物分析结果：{res}，{analysis}")
                animal_list[i] = res
    time.sleep(0.5)
    car.beep()
    car.beep()
    car.get_odometry(True)
    car.get_distance(True)
    return animal_list
