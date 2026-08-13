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


class LanePidCal:
    """
    车道PID控制器类

    该类用于车道保持控制，包含y轴和角度PID控制器。
    """

    def __init__(self, cfg_pid_y, cfg_pid_angle):
        """
        初始化车道PID控制器

        参数:
            cfg_pid_y: y轴PID控制器的配置参数
            cfg_pid_angle: 角度PID控制器的配置参数
        """
        # y_out_limit = 0.7
        # self.pid_y = PID(5, 0, 0)
        # self.pid_y.setpoint = 0
        # self.pid_y.output_limits = (-y_out_limit, y_out_limit)
        # print(cfg_pid_y)
        # print(cfg_pid_angle)
        self.pid_y = PID(**cfg_pid_y)
        # print(self.pid_y)

        angle_out_limit = 1.5
        self.pid_angle = PID(3, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)

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


class DetPidCal:
    """
    检测PID控制器类

    该类用于目标检测控制，包含y轴和角度PID控制器。
    """

    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        """
        初始化检测PID控制器

        参数:
            cfg_pid_y: y轴PID控制器的配置参数（可选）
            cfg_pid_angle: 角度PID控制器的配置参数（可选）
        """
        y_out_limit = 0.7
        self.pid_y = PID(0.3, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        angle_out_limit = 1.5
        self.pid_angle = PID(2, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)

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


class LocatePidCal:
    """
    定位PID控制器类

    该类用于位置定位控制，包含x轴和y轴PID控制器。
    """

    def __init__(self):
        """
        初始化定位PID控制器

        初始化x轴和y轴的PID控制器，设置默认参数和输出限制。
        """
        y_out_limit = 0.3
        self.pid_y = PID(0.5, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        x_out_limit = 0.3
        self.pid_x = PID(0.5, 0, 0)
        self.pid_x.setpoint = 0
        self.pid_x.output_limits = (-x_out_limit, x_out_limit)

    def set_target(self, x, y):
        """
        设置目标位置

        参数:
            x: x轴目标位置
            y: y轴目标位置
        """
        self.pid_y.setpoint = y
        self.pid_x.setpoint = x

    def get_out(self, error_x, error_y):
        """
        计算PID输出

        参数:
            error_x: x轴误差
            error_y: y轴误差

        返回:
            tuple: (x轴PID输出, y轴PID输出)
        """
        pid_y_out = self.pid_y(error_y)
        pid_x_out = self.pid_x(error_x)
        return pid_x_out, pid_y_out
