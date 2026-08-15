# -*- coding: utf-8 -*-
"""巡线/车道保持(LaneMixin): 前置摄像头车道闭环控制(从 motion.py 拆分而来)。"""
import time

# 方法默认参数用到的停止标志默认值(与 MyCar.STOP_PARAM 类属性保持一致)
STOP_PARAM = True


class LaneMixin:

    # correction CNN 叠加到角速度的参数(实车标定)
    _corr_threshold = 0.05   # |steer| 小于此值不启用(防抖)
    _corr_weight = 0.5       # steer 1.0 对应的角速度贡献

    def lane_base(self, speed, end_fuction, stop=STOP_PARAM):
        """
        车道保持基础方法(双模型版)

        使用前置摄像头进行车道检测和保持:
            - 转弯: lane 模型的 error_angle(d_e) 进 cfg_pid_angle -> 角速度
            - 居中: correction CNN 的 steer 叠加到角速度(打方向回正, 同时
              修正"不居中"与"不平行"); y_speed 横向通道不再使用
            - lane 模型的 error_y(中线误差, 单模型版曾用它做横向平移)弃用

        标定:
            _corr_threshold: |steer| 低于此值不叠加, 防抖(默认 0.05)
            _corr_weight:    steer 1.0 对应的角速度贡献(默认 0.5;
                             居中漂移 -> 调高, 跟线不稳 -> 调低, 抖动 -> 调大阈值)
        """
        # 速度分级: correction steer |s| >= _lane_err_max 时降到最低速
        err_max = getattr(self, "_lane_err_max", 0.3)
        v_min = getattr(self, "_lane_v_min", 0.15)
        v_max = max(speed, v_min)
        corr_threshold = getattr(self, "_corr_threshold", 0.05)
        corr_weight = getattr(self, "_corr_weight", 0.5)

        while True:
            if self._stop_flag:
                return

            # 只用转弯(d_e); error_y(d_a) 弃用, 横向通道置 0
            _, error_angle = self.get_lane_results()
            # 转弯通道: lane 模型 error_angle -> 角速度
            angle_speed = self.lane_pid_angle(-error_angle)
            # 居中通道: correction steer 叠加到角速度(打方向回正)
            correction_steer = self.get_correction_steer()
            if abs(correction_steer) > corr_threshold:
                angle_speed += correction_steer * corr_weight
            # correction steer 大小驱动纵向速度分级(误差大降速)
            k_err = max(0.0, 1.0 - abs(correction_steer) / err_max)
            run_speed = v_min + (v_max - v_min) * k_err
            self.set_velocity(run_speed, 0.0, angle_speed)
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
