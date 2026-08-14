# -*- coding: utf-8 -*-
"""基础移动原语(MoveMixin): 底层运动控制与坐标计算(从 motion.py 拆分而来)。"""
import math

# 方法默认参数用到的停止标志默认值(与 MyCar.STOP_PARAM 类属性保持一致)
STOP_PARAM = True

class MoveMixin:
    def move_base(self, sp, end_fuction, stop=STOP_PARAM):
        """
        基础移动方法
        设置车辆速度并持续移动，直到满足结束条件。
        参数:
            sp: 速度向量 [x, y, z]
            end_fuction: 结束条件函数，返回True时停止移动
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        self.set_velocity(sp[0], sp[1], sp[2])
        while True:
            if self._stop_flag:
                return
            if end_fuction():
                break
            self.set_velocity(sp[0], sp[1], sp[2])
        if stop:
            self.set_velocity(0, 0, 0)

    def move_time(self, sp, dur_time=1, stop=STOP_PARAM):
        """
        按时间移动
        以给定速度移动指定的时间。
        参数:
            sp: 速度向量 [x, y, z]
            dur_time: 移动时间（秒），默认为1
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        self.set_velocity_for_duration(sp[0], sp[1], sp[2], dur_time)
        if stop:
            self.stop()

    def move_distance(self, sp, dis=0.1, stop=STOP_PARAM):
        """
        按距离移动
        以给定速度移动指定的距离。
        参数:
            sp: 速度向量 [x, y, z]
            dis: 移动距离，默认为0.1
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        end_dis = self.get_distance() + dis

        def end_func():
            return self.get_distance() > end_dis

        self.move_base(sp, end_func, stop)

    def calculation_dis(self, pos_dst, pos_src):
        """
        计算两个坐标的距离
        计算两个二维坐标点之间的欧几里得距离。
        参数:
            pos_dst: 目标坐标 [x, y]
            pos_src: 源坐标 [x, y]

        返回:
            float: 两个坐标之间的距离
        """
        return math.sqrt(
            (pos_dst[0] - pos_src[0]) ** 2 + (pos_dst[1] - pos_src[1]) ** 2
        )
