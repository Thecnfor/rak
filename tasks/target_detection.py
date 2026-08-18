import time

from smartcar import PID
from smartcar.whalesbot.tools import CountRecord


def align_forward(car, delta_x=0.2, delta_y=None, sort_pos=(0.2, 0), time_out=4.0):
    """动物识别专用对齐: 只选列前(dx>=delta_x)目标, 绝不倒车。

    等价于 move_to_detection_target, 但:
      - 只选 dx >= delta_x 的目标(列后目标靠前进永远对不齐, 且不允许倒车)
      - 输出钳制: set_velocity 永不为负(物理上绝不倒车)
    仅本任务使用, 不影响其他任务的公共方法。
    """
    x_count = CountRecord(2)
    y_count = CountRecord(3)
    out_x = 0.0
    out_y = 0.0
    if car.arm.side == "RIGHT":
        kp_y, kp_x, ki_x = -0.2, -0.25, 0.03
    else:
        kp_y, kp_x, ki_x = 0.2, 0.08, 0.0

    pid_x = PID(kp_x, ki_x)
    pid_x.output_limits = (-0.15, 0.15)
    pid_x.setpoint = delta_x
    time_stop = time.time() + time_out
    while True:
        if car._stop_flag:
            car.set_velocity(0, 0, 0)
            car.arm.x_speed(0)
            return -1, "None"

        dets = car.get_detection_results(sort_pos=sort_pos)
        dets = [d for d in dets if d[4] >= delta_x]   # 只向前: 丢弃列后目标

        if len(dets) > 0:
            det = dets[0]
            dx, dy = det[4:6]
            err_x = delta_x - dx
            out_x = 0.0 if abs(err_x) < 0.015 else -pid_x(dx)
            if out_x < 0:                              # 硬性: 绝不倒车
                out_x = 0.0
            out_y = 0.0 if delta_y is None else kp_y * (dy - delta_y)

            flag_x = x_count(abs(err_x) < 0.06)
            flag_y = y_count(abs(dy) < 0.02) if delta_y is not None else True
            if flag_x:
                out_x = 0
            if flag_y:
                out_y = 0
            if flag_x and flag_y:
                car.set_velocity(0, 0, 0)
                car.arm.x_speed(0)
                return det[0], det[2]
        else:
            x_count(False)
            y_count(False)
        car.set_velocity(out_x, 0, 0)
        car.arm.x_speed(out_y)
        time.sleep(0.05)

        if time.time() > time_stop:
            car.set_velocity(0, 0, 0)
            car.arm.x_speed(0)
            return (None, None)


def run(car) -> list:
    animal_list = [0, 0, 0, 0]
    car._lane_v_min = 0.20
    car.arm.move_y_position(0)
    car.arm.move_x_position(-0.2)
    car.get_distance(True)
    time.sleep(0.5)

    for i in range(4):
        cls_id, label = align_forward(car, delta_x=0.2, delta_y=None, sort_pos=(0.2, 0))
        time.sleep(0.2)
        if label == "animal":
            res, analysis = car.animal_image_analysis()
            if res is not None:
                car.beep()
                print(f"第{i+1}个动物分析结果：{res}，{analysis}")
                animal_list[i] = res
        time.sleep(0.2)
        if i < 1:
            car.lane_dis_offset(speed=0.2, dis_hold=0.13)
        else:
            car.move_distance([0.2, 0, 0], dis=0.13)

    time.sleep(0.5)
    car.beep()
    car.beep()
    car.get_odometry(True)
    car.get_distance(True)
    return animal_list
