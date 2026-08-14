# -*- coding: utf-8 -*-
"""PID 控制器集合类（从 car.py 拆分而来）。"""
from smartcar import PID


# 两个pid集合成一个
class PidCal2:
    """
    PID控制器集合类

    该类包含两个PID控制器，分别用于y轴和角度控制。
    """

    def __init__(self, cfg_pid_y, cfg_pid_angle):
        """
        初始化PID控制器集合

        参数:
            cfg_pid_y: y轴PID控制器的配置参数
            cfg_pid_angle: 角度PID控制器的配置参数
        """
        self.pid_y = PID(**cfg_pid_y)
        self.pid_angle = PID(**cfg_pid_angle)

    def get_out(self, error_y, error_angle):
        """
        计算PID输出

        参数:
            error_y: y轴误差
            error_angle: 角度误差

        返回:
            tuple: (y轴PID输出, 角度PID输出)
        """
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out
