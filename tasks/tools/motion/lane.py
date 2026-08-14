# -*- coding: utf-8 -*-
"""巡线/车道保持(LaneMixin): 前置摄像头车道闭环控制(从 motion.py 拆分而来)。"""
import time

# 方法默认参数用到的停止标志默认值(与 MyCar.STOP_PARAM 类属性保持一致)
STOP_PARAM = True


class LaneMixin:

    def lane_base(self, speed, end_fuction, stop=STOP_PARAM):
        """
        车道保持基础方法

        使用前置摄像头进行车道检测和保持, 根据中线误差/转弯误差调整车辆。
        error 为中线误差(道路正中相对车头中线的横向偏差), angle 为转弯误差。

        控制策略:
            - 中线误差大时降速(弯道更稳), 误差小时全速直行;
            - 角度通道对 -angle 走 PID(微分对测量值差分), 抑制摆动。
        """
        # 速度分级: 中线误差 |error| >= _lane_err_max 时降到最低速
        err_max = getattr(self, "_lane_err_max", 0.3)
        v_min = getattr(self, "_lane_v_min", 0.15)
        v_max = max(speed, v_min)

        while True:
            if self._stop_flag:
                return

            error_y, error_angle = self.get_lane_results()
            y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
            # 中线误差驱动的纵向速度分级(误差大降速)
            k_err = max(0.0, 1.0 - abs(error_y) / err_max)
            run_speed = v_min + (v_max - v_min) * k_err
            self.set_velocity(run_speed, y_speed, angle_speed)
            if end_fuction():
                break
        if stop:
            self.stop()

    def lane_time(self, speed, time_dur, stop=STOP_PARAM):
        """
        车道保持定时方法

        使用前置摄像头进行车道保持，持续指定的时间。

        参数:
            speed: 行驶速度
            time_dur: 持续时间（秒）
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        time_end = time.time() + time_dur

        def end_fuction():
            return time.time() > time_end

        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis(self, speed, dis_end, stop=STOP_PARAM):
        """
        车道保持定距方法

        使用前置摄像头进行车道保持，直到行驶距离超过指定值。

        参数:
            speed: 行驶速度
            dis_end: 目标距离
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """

        # lambda重新endfunction
        def end_fuction():
            return self.get_distance() > dis_end

        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis_offset(self, speed, dis_hold, stop=STOP_PARAM):
        """
        车道保持距离偏移方法

        使用前置摄像头进行车道保持，行驶指定的距离偏移量。

        参数:
            speed: 行驶速度
            dis_hold: 距离偏移量
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        dis_start = self.get_distance()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, stop=stop)
