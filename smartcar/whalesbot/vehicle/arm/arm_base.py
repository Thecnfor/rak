#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
机械臂控制模块

该模块实现了机械臂的运动控制, 包括竖直方向、水平方向的移动, 以及手部的控制。
"""

import math
import time
import numpy as np
import yaml
import os
import sys
from threading import Thread
from typing import Union

# 添加上本地目录
dir_this = os.path.abspath(os.path.dirname(__file__))
sys.path.append(dir_this)
# 添加上两层目录
dir_root = os.path.abspath(os.path.join(dir_this, '..', '..'))
sys.path.append(dir_root)

# 导入自定义模块
from ...tools import get_yaml, limit_val, CountRecord, PID, logger
from .. import (
    AnalogInput, MotorWrap, Key4Btn, ServoPwm,
    ServoBus, StepperWrap, PoutD
)

# 常量定义



POSITION_ERROR_THRESHOLD = 4e-4 # 位置误差阈值
STOP_CHECK_THRESHOLD = 1e-10 # 停止检查阈值


def get_path_relative(*args):
    """
    获取相对路径

    Args:
        *args: 路径组件

    Returns:
        str: 完整的绝对路径
    """
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)


class ArmController:
    """
    机械臂控制类, 负责机械臂的运动控制和状态管理

    Attributes:
        config: 配置参数
        motor_y: 竖直方向步进电机
        motor_x: 水平方向电机
        hand_servo: 手部舵机
        arm_servo: 手臂舵机
        pump: 气泵控制
        valve: 阀门控制
        y_pose_now: 当前竖直位置
        x_pose_now: 当前水平位置
        side: 机械臂方向
    """

    def __init__(self) -> None:
        """
        初始化机械臂控制类
        """
        self.yaml_path = get_path_relative("arm_cfg.yaml")

        with open(self.yaml_path, 'r') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        
        '''机械臂的长度'''
        self.arm_length: float = self.config["arm_length"]
        # 初始化各部分参数
        self.y_params_init(**self.config["vert_cfg"])
        self.x_params_init(**self.config["horiz_cfg"])
        self.hand_params_init(**self.config["hand_cfg"])
        self.position_params_init(**self.config["pos_cfg"])


    def y_params_init(self, motor, limit_port, pid, threshold):
        """
        初始化竖直方向电机参数

        Args:
            motor: 电机配置
            limit_port: 限位传感器端口
            pid: PID参数
            threshold: 位置阈值
        """
        self.motor_y = StepperWrap(**motor)
        self.y_limit_sensor = AnalogInput(limit_port)

        self.y_pose_start = self.motor_y.get_dis()
        self.y_pose_now = 0
        self.y_pid = PID(**pid)
        self.y_velocity_limit = pid['output_limits']
        self.y_distance_change = 0
        self.y_threshold = threshold  # 竖直位置阈值
        self.y_pose_last = 0

        self.y_pid_flag = CountRecord(5)
        self.y_stop_flag = CountRecord(10)

    def y_reset_check(self):
        """
        检查竖直方向是否到达限位

        Returns:
            bool: 是否到达限位
        """
        return self.y_limit_sensor.read() > 1000  # 磁敏传感器的值大于1000时, 则认为到达限位位置

    def y_stop_check(self):
        """
        检查竖直方向是否停止

        Returns:
            bool: 是否停止
        """
        return self.y_stop_flag(
            abs(self.y_distance_change) < STOP_CHECK_THRESHOLD
        )
    def y_get_position(self):
        self.y_pose_now = (
            self.motor_y.get_dis() - self.y_pose_start
        )
        return self.y_pose_now

    def y_pid_moveto(self, target_pose):
        """
        使用PID控制竖直方向移动

        Args:
            target_pose: 目标位置 (单位: m)

        Returns:
            bool: 是否到达目标位置
        """
        # 记录当前位置, 并更新上次的位置
        self.y_pose_now = (
            self.motor_y.get_dis() - self.y_pose_start
        )
        self.y_distance_change = (
            self.y_pose_now - self.y_pose_last
        )
        self.y_pose_last = self.y_pose_now

        error = target_pose - self.y_pose_now
        velocity = self.y_pid(self.y_pose_now)

        self.y_speed(velocity)

        if self.y_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            return True
        else:
            return False

    def reset_y(self):
        """
        重置竖直方向位置
        """
        self.y_pid.setpoint = -0.25
        while True:
            if self.y_pid_moveto(-0.25):
                break
            if self.y_reset_check():
                self.y_pose_start = self.motor_y.get_dis()
                self.y_pose_now = 0
                break
        self.y_speed(0)

    def move_y_position(self, target):
        """
        移动竖直方向指定距离

        Args:
            target: 目标位置
        """
        self.y_pid.setpoint = target
        while True:
            if self.y_pid_moveto(target):
                logger.info(f"移动到高度{target}")
                break
            if self.y_stop_check():
                logger.info(f"移到高度{target}过程中检测到停止")
                break
        self.y_speed(0)

    def x_params_init(self, motor, pid, threshold):
        """
        初始化水平方向电机参数

        Args:
            motor: 电机配置
            pid: PID参数
            threshold: 位置阈值
        """
        # 定义水平移动电机,PID参数
        self.motor_x = MotorWrap(**motor)
        self.x_pid = PID(**pid)
        self.x_velocity_limit = pid['output_limits']
        self.x_pose_start = self.motor_x.get_dis()
        self.x_pose_now = 0
        self.x_threshold = threshold
        self.x_pose_last = 0

        self.x_distance_change = 0

        self.x_stop_flag = CountRecord(10)
        self.x_pid_flag = CountRecord(5)

    def x_stop_check(self):
        """
        检查水平方向是否停止

        Returns:
            bool: 是否停止
        """
        return self.x_stop_flag(
            abs(self.x_distance_change) < STOP_CHECK_THRESHOLD
        )
    def x_get_position(self):
        self.x_pose_now = self.motor_x.get_dis() - self.x_pose_start
        return self.x_pose_now

    def x_pid_moveto(self, target_pose):
        """
        使用PID控制水平方向移动

        Args:
            target_pose: 目标位置

        Returns:
            bool: 是否到达目标位置
        """
        self.x_pose_now = (
            self.motor_x.get_dis() - self.x_pose_start
        )
        self.x_distance_change = (
            self.x_pose_now - self.x_pose_last
        )
        self.x_pose_last = self.x_pose_now
        error = target_pose - self.x_pose_now

        velocity = self.x_pid(self.x_pose_now)

        self.x_speed(velocity)

        if self.x_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            return True
        else:
            return False

    def move_x_position(self, target, out_time = 6.0):
        """
        移动水平方向指定位置

        Args:
            target: 目标位置
        """
        end_time = time.time()+out_time
        self.x_pid.setpoint = target
        while True:
            if time.time() > end_time:
                break
            if self.x_pid_moveto(target):
                break
            if self.x_stop_check():
                dis = self.motor_x.get_dis()
                if dis <0.15:
                    self.x_pose_start = dis
                else:
                    self.x_pose_start = dis - 0.31
                break
            time.sleep(0.05)
        self.x_speed(0)



    def reset_x(self):
        """
        重置水平方向位置
        """
        target = -0.33
        self.x_pid.output_limits = (-0.06, 0.06)
        self.x_pid.setpoint = target
        while True:
            if self.x_pid_moveto(target):
                break
            if self.x_stop_check():
                self.x_pose_start = self.motor_x.get_dis()
                self.x_pose_now = 0
                self.x_pose_last = 0
                break
        self.x_speed(0)

    def hand_params_init(self, hand, hand2, grap):
        """
        初始化手部参数

        Args:
            hand: 手臂舵机配置
            hand2: 手部舵机配置
            grap: 抓取机构配置
        """
        self.hand_servo = ServoPwm(hand2["port"], mode=hand2["mode"])
        self.hand_angle_list2 = hand2["angle_list"]
        self.arm_servo = ServoBus(hand["port"])
        self.hand_angle_list = hand["angle_list"]
        self.pump = PoutD(grap["port_pump"])
        self.valve = PoutD(grap["port_valve"])

    def grasp(self, value: bool):
        """
        控制抓取机构

        Args:
            value: 抓取状态, True为抓取, False为释放
        """
        self.pump.set(not value)
        self.valve.set(value)


    def position_params_init(self, pose_enable, pose_horiz, pose_vert, side):
        """
        初始化位置参数

        Args:
            pose_enable: 是否启用位置
            pose_horiz: 水平位置
            pose_vert: 竖直位置
            side: 方向
        """
        self.pose_enable = pose_enable
        self.y_pose_start = (
            self.motor_y.get_dis() - pose_vert
        )
        self.y_pose_now = pose_vert
        self.x_pose_start = (
            self.motor_x.get_dis() - pose_horiz
        )
        self.x_pose_now = pose_horiz
        self.side = side

    def save_config(self, pose_enable=True):
        """
        保存配置到YAML文件

        Args:
            pose_enable: 是否启用位置
        """
        self.config["pos_cfg"] = {
            "pose_enable": pose_enable,
            "pose_horiz": self.x_pose_now,
            "pose_vert": self.y_pose_now,
            "side": self.side
        }
        with open(self.yaml_path, 'w') as stream:
            yaml.dump(self.config, stream, sort_keys=False)

    def y_speed(self, velocity):
        """
        设置竖直方向速度

        Args:
            velocity: 速度值
        """
        velocity = limit_val(velocity, *self.y_velocity_limit)
        self.motor_y.set_velocity(velocity)

    def x_speed(self, velocity):
        """
        设置水平方向速度

        Args:
            velocity: 速度值
        """
        velocity = limit_val(velocity, *self.x_velocity_limit)
        self.motor_x.set_linear(velocity)

    def set_position_start(self, y_position):
        """
        设置起始位置

        Args:
            y_position: 竖直位置
        """
        self.y_pose_start = self.y_pose_now
        self.x_pose_start = self.x_pose_now
        self.save_config()

    def set_manually(self):
        """
        使用【4键】控制机械臂
        """
        self.key = Key4Btn(4)
        logger.info("Using 4 keys to control arm...")
        while True:
            value = self.key.get_key()
            if value == 1:
                self.y_speed(0.1)  # 向上
            elif value == 3:
                self.y_speed(-0.1)  # 向下
            elif value == 4:
                self.x_speed(0.1)  # 向右
            elif value == 2:
                self.x_speed(-0.1)  # 向左
            else:
                self.x_speed(0)
                self.y_speed(0)

    def reset_position(self):
        """
        重置机械臂位置
        """
        thread_reset_y = Thread(target=self.reset_y)
        thread_reset_x = Thread(target=self.reset_x)

        self.set_hand_angle("UP")
        self.set_arm_angle("RIGHT")
        thread_reset_y.daemon = True
        thread_reset_x.daemon = True
        thread_reset_y.start()
        thread_reset_x.start()
        thread_reset_y.join()
        thread_reset_x.join()
        self.x = 0
        self.y = 0
        self.save_config()

    def switch_side(self, side):
        """
        切换机械臂方向

        Args:
            side: 机械臂的方向, LEFT、RIGHT或MID
        """
        if self.side != side:
            self.side = side
            logger.info(f"Changing side to {self.side}")
        else:
            return
        angle_target = self.hand_angle_list[side]
        self.set_arm_angle(angle_target, 80)
        time.sleep(0.5)

    
    
    def set_arm_angle(self, angle: Union[str, int] = "RIGHT", speed=80):
        """
        设置机械臂角度

        Args:
            angle: 目标角度，可以是字符串（"LEFT", "MID", "RIGHT"）或数字
            speed: 速度
        """
        _angle = angle
        if isinstance(_angle, str):
            self.side = _angle
            assert _angle in ("LEFT", "MID", "RIGHT"), "Direction should be LEFT, MID, or RIGHT"
            _angle = self.hand_angle_list[_angle]
        self._arm_angle_last = _angle
        self.arm_servo.set_angle(_angle, speed)

    def set_hand_angle(self, angle: Union[str, int] = "UP", speed=80):
        """
        设置机械臂手角度

        Args:
            angle: 目标角度，可以是字符串（"UP", "MID", "DOWN"）或数字
            speed: 速度
        """
        if isinstance(angle, str):
            assert angle in ("UP","MID","DOWN"), "Direction should be UP, MID, or DOWN"
            angle = self.hand_angle_list2[angle]
        self._hand_angle_last = angle
        self.hand_servo.set_angle(angle, speed)

    def go_for(self, x_offset, y_offset, time_run=None, speed=[0.15, 0.04]):
        """
        移动机械臂到当前位置的相对量

        Args:
            x_offset: 水平偏移
            y_offset: 竖直偏移
            time_run: 运行时间
            speed: 速度 [水平速度, 竖直速度]
        """
        x_pos = self.x_pose_now + x_offset
        y_pos = self.y_pose_now + y_offset
        self.goto_position(x_pos, y_pos, time_run, speed)
    
    def goto_position(self, x=None, y=None,time_run=None, speed= [0.15, 0.04]):
        """
        移动到指定机械臂位置

        Args:
            x: 水平位置
            y: 竖直位置
            time_run: 运行时间
            speed: 速度 [水平速度, 竖直速度]
        """

        # 控制上下限
        x_pos = limit_val(
            x,
            self.x_threshold[0],
            self.x_threshold[1]
        )
        y_pos = limit_val(
            y,
            self.y_threshold[0],
            self.y_threshold[1]
        )

        # 获取结束时间和对应速度
        time_start = time.time()
        if time_run is not None:
            assert isinstance(time_run, (int, float)), "Time must be a number"
            # 根据时间求速度
            time_end = time_start + time_run
            y_time = time_run
            x_time = time_run
        elif speed is not None:
            # 根据速度求时间
            if isinstance(speed, (int, float)):
                speed_x = speed
                speed_y = speed
            elif isinstance(speed, (list, tuple)):
                speed_x = speed[0]
                speed_y = speed[1]
            else:
                logger.error("Invalid speed argument")
                return
            x_time = abs(
                x_pos - self.x_pose_now
            ) / speed_x
            y_time = abs(
                y_pos - self.y_pose_now
            ) / speed_y
            time_run = max(x_time, y_time)
        else:
            logger.error("Either time_run or speed must be provided")
            return
        # 超时时间
        time_end = time_start + time_run

        # 定义结束标志和到达位置标记量
        if y is None:
            y_flag = True
        else:
            y_flag = False
        
        if x is None:
            x_flag = True
        else:
            x_flag = False

        # 获取对应的速度和pid位置
        if y_time < 0.1:
            speed_y = 0.1
            y_flag = True
        else:
            speed_y = abs(
                y_pos - self.y_pose_now
            ) / y_time

        self.y_pid.setpoint = y_pos
        self.y_pid.output_limits = (-speed_y, speed_y)

        if x_time < 0.1:
            speed_x = 0.1
            x_flag = True
        else:
            speed_x = abs(
                x_pos - self.x_pose_now
            ) / x_time

        self.x_pid.setpoint = x_pos
        self.x_pid.output_limits = (
            -speed_x, speed_x
        )

        # 开始移动前, 位置信息定义, 如果中间中断此时位置信息无用
        self.save_config(pose_enable=False)

        while True:
            # 到达结束标志结束
            if y_flag and x_flag:
                break
            # 获取剩余时间
            time_remain = time_end - time.time()
            # 超时处理
            if time_remain < -3:
                logger.warning("Timeout")
                # 超时停止
                self.x_speed(0)
                self.y_speed(0)
                break
            if not y_flag:
                if self.y_pid_moveto(y_pos):
                    self.y_speed(0)
                    y_flag = True

                # 重置初始化位置
                if self.y_reset_check():
                    if self.y_pid.setpoint <= self.y_pose_now:
                        y_flag = True
                        self.y_speed(0)
                    self.y_pose_start = self.motor_y.get_dis()
                    self.y_pose_now = 0
                    self.save_config()

            if not x_flag:
                if self.x_pid_moveto(x_pos):
                    self.x_speed(0)
                    x_flag = True

        self.save_config()
        # logger.debug(
        #     f"机械臂移动完成，当前位置状态: x: {self.x_pose_now:.4f}, y: {self.y_pose_now:.4f}, hand: {self.side}。 "
        # )
    def set_arm_pose(self,x=None,y=None,arm = None,hand = None):
        '''
        设置机械臂的位位姿

        Args:
            x: 水平位置
            y: 竖直位置
            arm: 手臂角度，可以是字符串（"LEFT", "MID", "RIGHT"）或数字
            hand: 手部角度，可以是字符串（"UP", "MID", "DOWN"）或数字
        
        '''
        self.goto_position(x, y)
        # time.sleep(0.2)
        if arm is not None:
            self.set_arm_angle(arm)
            time.sleep(1)
        if hand is not None:
            self.set_hand_angle(hand)

    # ==================== 便捷属性接口 ====================
    @property
    def y(self) -> float:
        """获取当前竖直位置（单位：mm）"""
        return self.y_get_position() * 1000.0

    @y.setter
    def y(self, mm: float):
        """设置目标竖直位置（单位：mm）"""
        self.move_y_position(mm / 1000.0)

    @property
    def x(self) -> float:
        """获取当前水平位置（单位：mm）"""
        return self.x_get_position() * 1000.0

    @x.setter
    def x(self, mm: float):
        """设置目标水平位置（单位：mm）"""
        self.move_x_position(mm / 1000.0)

    @property
    def angle(self) -> float:
        """获取手臂舵机当前角度"""
        return self._arm_angle_last if hasattr(self, '_arm_angle_last') else 0

    @angle.setter
    def angle(self, val: Union[str, int]):
        """设置手臂舵机角度"""
        self.set_arm_angle(val)

    @property
    def hand_angle(self) -> float:
        """获取手部舵机当前角度"""
        return self._hand_angle_last if hasattr(self, '_hand_angle_last') else 0

    @hand_angle.setter
    def hand_angle(self, val: Union[str, int]):
        """设置手部舵机角度"""
        self.set_hand_angle(val)


if __name__ == '__main__':
    arm = ArmController()
    print(f"机械臂长度: {arm.arm_length}")
    arm.reset_x()
    arm.x_move_to_position(0.1)
    print(arm.x_get_positon())
    arm.x_move_to_position(0.2)
    print(arm.x_get_positon())
    arm.x_move_to_position(0.3)
    print(arm.x_get_positon())   

    # start_time = time.time()
    # # arm.grasp(True)
    # arm.reset_position()
    # arm.goto_position(0.15, 0.1)
    # # time.sleep(1)
    # arm.set_arm_angle("LEFT")
    # time.sleep(1)
    # arm.set_hand_angle("DOWN")
    # # arm.grasp(False)
    
    # print(f"移动时间: {time.time() - start_time:.4f}秒")
    # print(f"x: {arm.x_pose_now:.4f}, y: {arm.y_pose_now:.4f}")
